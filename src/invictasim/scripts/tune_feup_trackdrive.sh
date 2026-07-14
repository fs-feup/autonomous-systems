#!/bin/bash
# Script to convert a FEUP_trackdrive rosbag to CSV and then tune the vehicle model

WORKSPACE_DIR="/home/leandro/Uni/FS/autonomous-systems"
cd "$WORKSPACE_DIR"

ROSBAG_PATH="FEUP_trackdrive"

if [ -n "$1" ]; then
    ROSBAG_PATH="$1"
fi

if [ ! -e "$ROSBAG_PATH" ]; then
    echo "Warning: Rosbag path '$ROSBAG_PATH' does not exist. Please provide the correct path as an argument."
    echo "Usage: ./src/invictasim/scripts/tune_feup_trackdrive.sh <path_to_FEUP_trackdrive_rosbag>"
    exit 1
fi

CONTAINER_ID=$(docker ps -q | head -n 1)
if [ -z "$CONTAINER_ID" ]; then
    echo "Error: No docker container found. Please start the devcontainer."
    exit 1
fi
echo "Using Docker container: $CONTAINER_ID"
docker exec -w /home/ws "$CONTAINER_ID" bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && python3 src/invictasim/scripts/rosbag_to_csv.py '$ROSBAG_PATH' --output FEUP_trackdrive.csv"

if [ $? -ne 0 ]; then
    echo "Failed to create CSV from rosbag."
    exit 1
fi

echo "Running optimizer with the generated CSV..."
./src/invictasim/scripts/run_optimizer.sh config/invictasim/tuning/02_feup_trackdrive_tuning.yaml
