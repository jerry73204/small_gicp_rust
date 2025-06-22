#!/bin/bash
set -e

# Update package list
sudo apt-get update

# Install C++ build tools and dependencies
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libomp-dev \
    libtbb-dev \
    pkg-config

# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"

# Install nightly toolchain for formatting
rustup install nightly
rustup component add rustfmt --toolchain nightly

# Install Python dependencies (for testing Python bindings)
sudo apt-get install -y \
    python3-dev \
    python3-pip \
    pybind11-dev
