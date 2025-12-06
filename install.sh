#!/bin/bash

set -euo pipefail



# Check if mamba command exists
if ! command -v mamba &> /dev/null; then
    echo "mamba not found."
    echo ""
    read -p "Would you like to install mamba from miniforge? (y/n) " -n 1 -r
    echo ""

    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Installation cancelled. Please install mamba manually to continue."
        exit 1
    fi

    echo "Installing miniforge..."

    # Detect OS and architecture
    OS=$(uname)
    ARCH=$(uname -m)

    # Map architecture names
    case $ARCH in
        x86_64)
            ARCH_NAME="x86_64"
            ;;
        arm64|aarch64)
            ARCH_NAME="arm64"
            ;;
        *)
            echo "Unsupported architecture: $ARCH"
            exit 1
            ;;
    esac

    # Download miniforge installer
    INSTALLER_URL="https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-${OS}-${ARCH_NAME}.sh"
    INSTALLER_FILE="Miniforge3-${OS}-${ARCH_NAME}.sh"

    echo "Downloading miniforge installer..."
    curl -L -O "$INSTALLER_URL" || {
        echo "Failed to download miniforge installer."
        exit 1
    }

    echo "Running installer..."
    bash "$INSTALLER_FILE" -b -p "$HOME/miniforge3" || {
        echo "Failed to install miniforge."
        rm -f "$INSTALLER_FILE"
        exit 1
    }

    # Clean up installer
    rm -f "$INSTALLER_FILE"

    # Initialize conda
    echo "Initializing conda..."
    "$HOME/miniforge3/bin/conda" init bash zsh || {
        echo "Warning: Failed to initialize conda. You may need to run 'conda init' manually."
    }

    # Add to PATH for current session
    export PATH="$HOME/miniforge3/bin:$PATH"

    echo ""
    echo "Miniforge installed successfully!"
    echo "Please restart your terminal or run: source ~/.bashrc (or ~/.zshrc)"
    echo ""
else
    echo "mamba is available."
fi

# Install piperlib dependencies
mamba env create -f environment.yml

# Set the PIPERLIB_CONDA_ENV environment variable, activate env only to set the environment variable
mamba activate piperlib && export PIPERLIB_CONDA_ENV=$CONDA_PREFIX && mamba deactivate

# Install the ruckig library
echo "Installing the ruckig library..."
cd ruckig
mkdir -p build && cd build
cmake ..
make -j
sudo make install
cd ../..

# Install the piperlib library
echo "Ready to install the piperlib library..."
uv pip install -e .
