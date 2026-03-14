#!/usr/bin/env bash

set -e  # Exit immediately if anything fails

echo "=== Setting up acados ==="

# ------------------------------------------------------------------
# 1. Install system dependencies
# ------------------------------------------------------------------
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libblas-dev \
    liblapack-dev \
    gfortran \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-numpy \
    swig

# Optional but recommended (QP solvers & performance)
sudo apt-get install -y \
    libopenblas-dev

# ------------------------------------------------------------------
# 2. Build acados
# ------------------------------------------------------------------
ACADOS_DIR="$(pwd)/ext/acados"

# We proactively fix ownership to ensure the current user owns the folder.
echo "Ensuring ownership of ${ACADOS_DIR}..."
sudo chown -R $USER:$USER "${ACADOS_DIR}/include/acados"

echo "Using acados at: ${ACADOS_DIR}"

cd "${ACADOS_DIR}"

# Update submodules inside acados
git submodule update --init --recursive

# Clean previous builds if any
sudo rm -rf build
sudo rm -rf include
sudo rm -rf lib
sudo rm -rf bin
sudo rm -rf share
mkdir build
cd build

cmake .. \
    -DACADOS_WITH_QPOASES=ON \
    -DACADOS_WITH_OSQP=ON \
    -DACADOS_WITH_OPENMP=OFF \
    -DBLASFEO_TARGET=X64_INTEL_HASWELL \
    -DHPIPM_TARGET=GENERIC \
    -DCMAKE_BUILD_TYPE=Release

make -j$(nproc)
make install

# ------------------------------------------------------------------
# 3. Python interface
# ------------------------------------------------------------------
cd "${ACADOS_DIR}/interfaces/acados_template"

pip3 install --user -e .


# ------------------------------------------------------------------
# 4. Environment variables
# ------------------------------------------------------------------
echo "Configuring environment variables..."

if ! grep -q "ACADOS_SOURCE_DIR" ~/.bashrc; then
    echo "export ACADOS_SOURCE_DIR=${ACADOS_DIR}" >> ~/.bashrc
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/ws/ext/acados/lib" >> ~/.bashrc
    echo "export PYTHONPATH=\$PYTHONPATH:${ACADOS_DIR}/interfaces/acados_template" >> ~/.bashrc
fi

# Apply immediately for current shell
export ACADOS_SOURCE_DIR="${ACADOS_DIR}"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:/home/ws/c_generated_code
export PYTHONPATH=$PYTHONPATH:${ACADOS_DIR}/interfaces/acados_template

echo "=== acados setup complete ==="
