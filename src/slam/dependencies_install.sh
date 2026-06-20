#!/usr/bin/env bash
set -euo pipefail

# For Graph SLAM, needed for GTSAM.
sudo apt-get install libboost-all-dev libmetis-dev # TODO: put all dependencies in files like this and make the main script call these files

# Add this library path to interactive shells without duplicating the entry.
if ! grep -Fxq 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' ~/.bashrc; then
  echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
fi
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
GTSAM_DIR="${REPO_ROOT}/ext/gtsam"
BUILD_DIR="${GTSAM_DIR}/build"

mkdir -p "${BUILD_DIR}"
cmake \
  -S "${GTSAM_DIR}" \
  -B "${BUILD_DIR}" \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DCMAKE_PREFIX_PATH="/usr/share/eigen3" \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_POSE3_EXPMAP=ON \
  -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -Wno-error -Wno-error=deprecated-copy -Wno-error=deprecated-declarations -Wno-error=cpp -Wno-error=pedantic"
# Build the GTSAM libraries only (no unit tests) and install.
make -C "${BUILD_DIR}" -j4
sudo make -C "${BUILD_DIR}" install -j4
