To build and run node, from /src:


colcon build --symlink-install
source ./install/setup.bash
ros2 launch yolov5_ros yolov5s_simple.launch.py
