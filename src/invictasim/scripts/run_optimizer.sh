#!/bin/bash
# Usage: ./run_optimizer.sh <config_yaml_path>

if [ -z "$1" ]; then
    echo "Usage: ./run_optimizer.sh <config_yaml_path>"
    echo "Example: ./run_optimizer.sh src/invictasim/config/tuning/my_custom_tune.yaml"
    exit 1
fi

echo "Sourcing ROS 2 workspace..."
source install/setup.bash

echo "Starting Vehicle Model Optimizer..."
./install/invictasim/lib/invictasim/vehicle_model_optimizer --config "$1" --data-dir src/invictasim/tuning_csvs
