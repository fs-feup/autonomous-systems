#!/usr/bin/env bash
set -e

REPO_ROOT="$(pwd)/../.." 
OSQP_DIR="${REPO_ROOT}/ext/osqp"
OSQP_EIGEN_DIR="${REPO_ROOT}/ext/osqp-eigen"
INSTALL_PREFIX="${REPO_ROOT}/ext/osqp/install"

# System deps
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libcgal-dev \
    libgsl-dev

# Clone if not present
[ -d "$OSQP_DIR/src" ] || git clone --recursive https://github.com/osqp/osqp.git "${OSQP_DIR}/src"
[ -d "$OSQP_EIGEN_DIR/src" ] || git clone https://github.com/robotology/osqp-eigen.git "${OSQP_EIGEN_DIR}/src"

# Build OSQP into ext/osqp/install
cd "${OSQP_DIR}/src"
mkdir -p build && cd build
cmake -G "Unix Makefiles" \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    ..
cmake --build . --target install

# Build osqp-eigen into same install prefix
cd "${OSQP_EIGEN_DIR}/src"
mkdir -p build && cd build
cmake \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}" \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    ..
make -j$(nproc)
make install