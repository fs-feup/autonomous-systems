rm -rf log/ install/ build/

colcon build --packages-up-to inspection ros_can rslidar_sdk

colcon build --packages-up-to perception

colcon build --packages-up-to ekf_state_est

MAKEFLAGS=-j4 colcon build --packages-up-to planning

MAKEFLAGS=-j4 colcon build --packages-up-to control launcher

source ./install/setup.bash

