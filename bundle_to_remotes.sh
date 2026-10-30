#!/bin/bash
# Build gb_receiver and copy its app image into the sibling remotes'
# firmware/receiver_fw/. They flash it into rx_fw and push it to older
# receivers over BLE.
#   bundle_to_remotes.sh [image.bin]   copy that image instead of building
#                                      (build_and_release.sh passes its release bin)
set -e
BIN=""
if [ -n "$1" ]; then
    BIN="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
fi
cd "$(dirname "$0")"

if [ -z "$BIN" ]; then
    if [ -n "$(git status --porcelain -- main components CMakeLists.txt)" ]; then
        echo "[WARN] gb_receiver has uncommitted changes; they go into the bundled image"
    fi
    # The remote build scripts export IDF_TARGET=esp32s3; the receiver is an esp32c3.
    env -u IDF_TARGET idf.py build
    BIN=build/gb_receiver.bin
fi

VERSION=$(grep -E '^#define FW_VERSION' main/version.h | sed -E 's/.*"([^"]+)".*/\1/')
for remote_fw in ../gb_remote/firmware ../gb_remote_dual_pro/firmware; do
    if [ -d "$remote_fw" ]; then
        mkdir -p "$remote_fw/receiver_fw"
        # The remote CMake flashes the one gb_receiver*.bin it finds, so drop older ones.
        rm -f "$remote_fw"/receiver_fw/gb_receiver*.bin
        cp "$BIN" "$remote_fw/receiver_fw/gb_receiver_v$VERSION.bin"
        echo "[INFO] Bundled receiver v$VERSION into $remote_fw/receiver_fw/"
    else
        echo "[WARN] $remote_fw not found, skipped"
    fi
done
