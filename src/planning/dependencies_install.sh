#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
OSQP_DIR="${REPO_ROOT}/ext/osqp"
OSQP_EIGEN_DIR="${REPO_ROOT}/ext/osqp-eigen"
INSTALL_PREFIX="${REPO_ROOT}/ext/osqp/install"

echo ">>> Installing OSQP dependencies for planning module..."
echo "    Repo root: ${REPO_ROOT}"

# Check submodules are initialized
if [ ! -f "${OSQP_DIR}/src/CMakeLists.txt" ]; then
    echo "ERROR: ext/osqp submodule not initialized."
    echo "Run: git submodule update --init --recursive"
    exit 1
fi
if [ ! -f "${OSQP_EIGEN_DIR}/src/CMakeLists.txt" ]; then
    echo "ERROR: ext/osqp-eigen submodule not initialized."
    echo "Run: git submodule update --init --recursive"
    exit 1
fi

# System deps
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libcgal-dev \
    libgsl-dev

# Build OSQP
cd "${OSQP_DIR}/src"
mkdir -p build && cd build
cmake -G "Unix Makefiles" \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    ..
cmake --build . --target install

# Build osqp-eigen
cd "${OSQP_EIGEN_DIR}/src"
mkdir -p build && cd build
cmake \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}" \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    ..
make -j$(nproc)
make install

echo ">>> OSQP installed successfully to ${INSTALL_PREFIX}"