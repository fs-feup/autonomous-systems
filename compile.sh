#!/bin/bash
set -e

# Check if install/setup.bash exists
if [ ! -f install/setup.bash ]; then
  echo "Error: install/setup.bash not found. Did you build the workspace before?"
  exit 1
fi

echo "Sourcing workspace..."
source install/setup.bash

echo "Building selected packages with Ninja..."
colcon build --packages-up-to perception slam velocity_estimation planning control launcher --cmake-args -G Ninja