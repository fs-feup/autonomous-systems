#!/bin/bash
# set -e
# # Check if install/setup.bash exists
# if [[ ! -f install/setup.bash ]]; then
# echo "Error: install/setup.bash not found. Did you build the workspace before?" >&2
# exit 1
# fi
# echo "Sourcing workspace..."
# source install/setup.bash
# echo "Building selected packages with -j2 parallelism..."
CMAKE_BUILD_PARALLEL_LEVEL=3 MAKEFLAGS=-j2 colcon build \
  --parallel-workers 2 \
  --event-handlers console_direct+ \
  --packages-up-to emergency_remote_control control pacsim planning ros_can \
  --cmake-args -G Ninja -DCMAKE_VERBOSE_MAKEFILE=ON
