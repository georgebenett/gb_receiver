#!/bin/bash
# Build and release script for gb_receiver project
# Creates zip packages and uploads them to GitHub releases

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Detect OS for sed compatibility
if [[ "$OSTYPE" == "darwin"* ]]; then
    sed_inplace() { sed -i '' "$@"; }
else
    sed_inplace() { sed -i "$@"; }
fi

# Function to print colored messages
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ] || [ ! -f "main/version.h" ]; then
    print_error "Must be run from project root directory"
    exit 1
fi

# Extract version from version.h
VERSION=$(grep -E '^#define FW_VERSION' main/version.h | sed -E 's/.*"([^"]+)".*/\1/')
if [ -z "$VERSION" ]; then
    print_error "Could not extract version from main/version.h"
    exit 1
fi

print_info "Building firmware version: $VERSION"

# Refuse to run with a dirty working tree — the tag we create must point at a
# commit whose source actually contains $VERSION, otherwise the released binary
# won't match the code at v$VERSION.
if ! git diff --quiet HEAD -- 2>/dev/null || [ -n "$(git ls-files --others --exclude-standard)" ]; then
    print_error "Working tree is dirty. Commit your version bump (and any other changes) before releasing."
    print_error "Run: git status"
    exit 1
fi

# Also require the current commit to be pushed, so origin has the commit the tag will reference.
if ! git diff --quiet "@{upstream}"..HEAD 2>/dev/null; then
    print_warn "Local commits are ahead of upstream. Push them before tagging so the release tag references a pushed commit."
    read -p "Continue anyway? [y/N]: " confirm_unpushed
    if [[ ! "$confirm_unpushed" =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check for required tools
if ! command -v gh &> /dev/null; then
    print_error "GitHub CLI (gh) is not installed. Install it from https://cli.github.com/"
    exit 1
fi

if ! command -v zip &> /dev/null; then
    print_error "zip command is not available. Please install it."
    exit 1
fi

# Check if gh is authenticated
if ! gh auth status &> /dev/null; then
    print_error "GitHub CLI is not authenticated. Run 'gh auth login'"
    exit 1
fi

# Get repository info
REPO=$(gh repo view --json nameWithOwner -q .nameWithOwner 2>/dev/null || echo "")
if [ -z "$REPO" ]; then
    print_warn "Could not detect GitHub repository. Will prompt for repository name."
    read -p "Enter repository (owner/repo): " REPO
fi

# Create temporary directory for artifacts
ARTIFACTS_DIR=$(mktemp -d)
trap "rm -rf $ARTIFACTS_DIR" EXIT

print_info "Artifacts will be stored in: $ARTIFACTS_DIR"

# Function to build a target
build_target() {
    local target=$1
    local config_file="sdkconfig.defaults"
    
    if [ -n "$target" ] && [ "$target" != "default" ]; then
        config_file="sdkconfig.defaults.$target"
    fi

    print_info "Building for ${target:-default} target..."

    # Use the appropriate config file if it exists
    if [ -f "$config_file" ]; then
        print_info "Using config file: $config_file"
        cp "$config_file" sdkconfig.defaults
    elif [ -f "sdkconfig.defaults" ]; then
        print_info "Using default sdkconfig.defaults"
    else
        print_warn "No sdkconfig.defaults found, using default configuration"
    fi

    # Enable ccache if available
    if command -v ccache &> /dev/null; then
        export IDF_CCACHE_ENABLE=1
    fi

    # Build
    BUILD_START=$(date +%s)
    idf.py build
    BUILD_END=$(date +%s)
    BUILD_TIME=$((BUILD_END - BUILD_START))
    print_info "Build completed in ${BUILD_TIME} seconds"

    # Find the main app binary (ESP-IDF creates it with the project name)
    # The project name is gb_receiver, so the binary should be gb_receiver.bin
    APP_BIN="build/gb_receiver.bin"

    if [ ! -f "$APP_BIN" ]; then
        # Try to find any .bin file in build root (excluding bootloader and partition table)
        APP_BIN=$(find build -maxdepth 1 -name "*.bin" -type f ! -name "bootloader.bin" ! -name "partition-table.bin" | head -1)
    fi

    if [ -z "$APP_BIN" ] || [ ! -f "$APP_BIN" ]; then
        print_error "Could not find main application binary in build directory"
        print_error "Expected: build/gb_receiver.bin"
        exit 1
    fi

    # Verify other required binaries exist
    if [ ! -f "build/bootloader/bootloader.bin" ]; then
        print_error "Could not find bootloader.bin"
        exit 1
    fi

    if [ ! -f "build/partition_table/partition-table.bin" ]; then
        print_error "Could not find partition-table.bin"
        exit 1
    fi

    # Package artifacts
    ZIP_NAME="receiver_v${VERSION}.zip"
    ZIP_PATH="$ARTIFACTS_DIR/$ZIP_NAME"

    print_info "Packaging ${target:-default} artifacts into $ZIP_NAME..."

    # Create a temporary staging directory for packaging
    STAGING_DIR=$(mktemp -d)

    # Copy and rename files to staging directory with correct names
    # Include .elf for coredump decoding (must match flashed binary to avoid SHA mismatch)
    cp "$APP_BIN" "$STAGING_DIR/gb_receiver.bin"
    cp "build/bootloader/bootloader.bin" "$STAGING_DIR/bootloader.bin"
    cp "build/partition_table/partition-table.bin" "$STAGING_DIR/partition-table.bin"
    if [ -f "build/gb_receiver.elf" ]; then
        cp "build/gb_receiver.elf" "$STAGING_DIR/gb_receiver.elf"
    fi

    # Create zip from staging directory (files will be at root level)
    cd "$STAGING_DIR"
    zip -q "$ZIP_PATH" \
        gb_receiver.bin \
        bootloader.bin \
        partition-table.bin
    if [ -f "gb_receiver.elf" ]; then
        zip -q "$ZIP_PATH" gb_receiver.elf
    fi
    cd - > /dev/null

    # Clean up staging directory
    rm -rf "$STAGING_DIR"

    # Verify zip was created
    if [ ! -f "$ZIP_PATH" ]; then
        print_error "Failed to create zip file"
        exit 1
    fi

    print_info "Created $ZIP_NAME ($(du -h "$ZIP_PATH" | cut -f1))"

    # Clean build directory for next target
    print_info "Cleaning build directory..."
    if [ -d "build" ]; then
        rm -rf build
    fi
}

# Build targets
# Find all sdkconfig.defaults.* files (excluding the base sdkconfig.defaults)
TARGETS=$(find . -maxdepth 1 -name "sdkconfig.defaults.*" -type f | sed 's|./sdkconfig.defaults.||' | sort)

if [ -n "$TARGETS" ]; then
    print_info "Found multiple build targets: $TARGETS"
    # Build each target-specific config
    for target in $TARGETS; do
        build_target "$target"
    done
else
    # Single build target - use default config
    print_info "Building single target with default configuration"
    build_target "default"
fi

# List created artifacts
print_info "Created artifacts:"
ls -lh "$ARTIFACTS_DIR"/*.zip

# Ask for confirmation before creating release
echo ""
print_warn "Ready to create GitHub release v$VERSION"
read -p "Continue? [y/N]: " confirm
if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    print_info "Cancelled. Artifacts are in: $ARTIFACTS_DIR"
    print_info "You can manually upload them later."
    exit 0
fi

# Create git tag if it doesn't exist and push
if ! git rev-parse "v$VERSION" >/dev/null 2>&1; then
    print_info "Creating git tag v$VERSION..."
    git tag -a "v$VERSION" -m "Release v$VERSION"
    print_info "Pushing tag v$VERSION..."
    git push origin "v$VERSION"
else
    print_info "Tag v$VERSION already exists; pushing if needed..."
    git push origin "v$VERSION" 2>/dev/null || true
fi

# Create GitHub release
print_info "Creating GitHub release v$VERSION..."

# Check if release already exists
if gh release view "v$VERSION" --repo "$REPO" &> /dev/null; then
    print_warn "Release v$VERSION already exists. Do you want to:"
    echo "  1) Delete and recreate it"
    echo "  2) Add artifacts to existing release"
    echo "  3) Cancel"
    read -p "Choice [1-3]: " choice

    case $choice in
        1)
            print_info "Deleting existing release..."
            gh release delete "v$VERSION" --repo "$REPO" --yes
            gh release create "v$VERSION" \
                --repo "$REPO" \
                --title "v$VERSION" \
                --notes "Firmware release v$VERSION" \
                "$ARTIFACTS_DIR"/*.zip
            ;;
        2)
            print_info "Uploading artifacts to existing release..."
            gh release upload "v$VERSION" \
                --repo "$REPO" \
                "$ARTIFACTS_DIR"/*.zip \
                --clobber
            ;;
        3)
            print_info "Cancelled"
            exit 0
            ;;
        *)
            print_error "Invalid choice"
            exit 1
            ;;
    esac
else
    # Create new release
    ARTIFACT_LIST=""
    for zip_file in "$ARTIFACTS_DIR"/*.zip; do
        if [ -f "$zip_file" ]; then
            filename=$(basename "$zip_file")
            ARTIFACT_LIST="${ARTIFACT_LIST}- \`${filename}\` - Receiver firmware build"$'\n'
        fi
    done

    gh release create "v$VERSION" \
        --repo "$REPO" \
        --title "v$VERSION" \
        --notes "Firmware release v$VERSION

## Artifacts
${ARTIFACT_LIST}

Each zip contains:
- \`gb_receiver.bin\` - Application binary
- \`bootloader.bin\` - Bootloader binary
- \`partition-table.bin\` - Partition table
- \`gb_receiver.elf\` - Application ELF (for coredump decoding)" \
        "$ARTIFACTS_DIR"/*.zip
fi

print_info "Release created successfully!"
print_info "View release at: https://github.com/$REPO/releases/tag/v$VERSION"

