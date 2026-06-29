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
    ninja-build \
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
WORKSPACE_DIR="$(pwd)"
ACADOS_SRC_DIR="${WORKSPACE_DIR}/ext/acados"
ACADOS_INSTALL_DIR="${WORKSPACE_DIR}/build/acados-install"

# We proactively fix ownership to ensure the current user owns the folder.
echo "Ensuring ownership of ${ACADOS_SRC_DIR}..."
# sudo chown -R $USER:$USER "${ACADOS_SRC_DIR}/include/acados"

echo "Using acados source at: ${ACADOS_SRC_DIR}"
echo "Installing acados to: ${ACADOS_INSTALL_DIR}"

mkdir -p "${ACADOS_SRC_DIR}"

cd "${ACADOS_SRC_DIR}"

# Update submodules inside acados
git submodule update --init --recursive

# Clean previous builds if any
sudo rm -rf build
sudo rm -rf "${ACADOS_INSTALL_DIR}"
mkdir build
cd build

cmake -S . -B build -G Ninja \
    -DACADOS_WITH_QPOASES=ON \
    -DACADOS_WITH_OSQP=ON \
    -DACADOS_WITH_OPENMP=ON \
    -DBLASFEO_TARGET=X64_INTEL_HASWELL \
    -DHPIPM_TARGET=GENERIC \
    -DCMAKE_BUILD_RPATH=\$ORIGIN \
    -DCMAKE_INSTALL_RPATH=\$ORIGIN \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
    -DCMAKE_INSTALL_PREFIX="${ACADOS_INSTALL_DIR}" \
    -DCMAKE_BUILD_TYPE=Release

cmake --build build --target install --parallel "$(nproc)"

# ------------------------------------------------------------------
# 3. Python interface
# ------------------------------------------------------------------
cd "${ACADOS_SRC_DIR}/interfaces/acados_template"

pip3 install --user -e .

# ------------------------------------------------------------------
# 4. Done
# ------------------------------------------------------------------
echo "=== acados setup complete ==="
echo ""
echo "You can now compile and run the control package."
echo ""
echo "Build the control package:"
echo "  colcon build --packages-select control --cmake-args -G Ninja"
echo ""
echo "The acados libraries are installed to:"
echo "  ${ACADOS_INSTALL_DIR}"
echo ""
echo "No additional environment setup is needed - CMakeLists will auto-detect."
