#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -z "${ROOT_WORKTREE_PATH:-}" ]; then
    ROOT_WORKTREE_PATH="$(cd "${SCRIPT_DIR}/.." && pwd)"
fi

PACKAGE_PATH="$ROOT_WORKTREE_PATH"
VENV_DIR_NAME=".like_keras_venv"
VENV_BASE_DIR="$(dirname "$PACKAGE_PATH")" # Make one level up to avoid cluttering the repository tree
VENV_PATH="$VENV_BASE_DIR/$VENV_DIR_NAME"

echo "Setting up worktree..."
echo "Package path: $PACKAGE_PATH"
echo "Virtual environment directory: $VENV_PATH"

# Create a dedicated virtual environment for this worktree
# This ensures each parallel agent has its own isolated environment while
# keeping the environment outside of the repository tree.
if [ -d "$VENV_PATH" ]; then
    echo "Virtual environment already exists at: $VENV_PATH"
else
    echo "Creating virtual environment for worktree..."
    python3 -m venv "$VENV_PATH"
fi

echo "Activating virtual environment: $VENV_PATH"
source "$VENV_PATH/bin/activate"

# Install Python package in editable mode
echo "Installing Python package from pyproject.toml..."
cd "$PACKAGE_PATH"
pip install --upgrade pip
pip install -e .


echo "Worktree setup complete!"

