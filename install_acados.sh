#!/bin/bash

# Install system dependencies
sudo apt-get update
sudo apt-get install -y git cmake make gcc g++ python3-venv python3-pip

# Clone Acados
if [ ! -d "acados" ]; then
    git clone https://github.com/acados/acados.git
    cd acados
    git submodule update --recursive --init
else
    cd acados
fi

# Build Acados
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make -j4
sudo make install
cd ..

# Install Python bindings
pip install -e interfaces/acados_template

# Set environment variables (add to .bashrc if needed)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/lib
export ACADOS_SOURCE_DIR=$(pwd)

echo "Acados installed successfully."
