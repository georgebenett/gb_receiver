#!/bin/bash

# Run script for read_data.py
# This script sets up the virtual environment, installs dependencies, and runs the application

VENV_DIR="venv"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if virtual environment exists
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
    
    # Activate virtual environment
    echo "Activating virtual environment..."
    source "$VENV_DIR/bin/activate"
    
    # Upgrade pip
    echo "Upgrading pip..."
    pip install --upgrade pip
    
    # Install dependencies
    echo "Installing dependencies..."
    pip install -r requirements.txt
else
    # Activate virtual environment
    source "$VENV_DIR/bin/activate"
    
    # Check if dependencies are installed
    if ! python -c "import PyQt6" 2>/dev/null; then
        echo "Installing missing dependencies..."
        pip install -r requirements.txt
    fi
fi

# Run the application
echo "Starting telemetry display..."
# Suppress harmless DBus portal warnings
export QT_QPA_PLATFORMTHEME=""
python "$SCRIPT_DIR/read_data.py"

