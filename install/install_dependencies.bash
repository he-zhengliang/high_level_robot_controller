#!/bin/bash

# List of Python packages to install
PYTHON_PACKAGES=("numpy" "pandas" "requests")

# List of system packages to check and install
SYSTEM_PACKAGES=("python3-venv" "python3-pip" "graphviz")

# Function to check and install system packages
install_system_packages() {
    echo "Checking and installing system packages..."
    for package in "${SYSTEM_PACKAGES[@]}"; do
        if dpkg -s $package &> /dev/null; then
            echo "$package is already installed"
        else
            echo "$package not found, installing..."
            sudo apt update
            sudo apt install -y $package
        fi
    done
}

# Function to create Python virtual environment
create_virtualenv() {
    echo "Creating Python virtual environment..."
    python3 -m venv venv

    if [ $? -eq 0 ]; then
        echo "Virtual environment created successfully."
    else
        echo "Failed to create virtual environment." >&2
        exit 1
    fi
}

# Function to install Python packages inside virtual environment
install_python_packages() {
    echo "Installing Python packages..."
    source venv/bin/activate

    for package in "${PYTHON_PACKAGES[@]}"; do
        pip install $package
        if [ $? -eq 0 ]; then
            echo "$package installed successfully."
        else
            echo "Failed to install $package." >&2
        fi
    done

    deactivate
}

# Main script execution
install_system_packages
create_virtualenv
install_python_packages

echo "Setup completed!"
