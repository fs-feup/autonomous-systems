# MockerNode package

This package is used to mock outputs from other packages. It has a central ROS2 node that deals with ROS2 communication and will have 4 additional files that 
separately mock each of the following subsystems: Perception, State Estimation, Path Planning and Driverless Control. The configuration should be done by launching
with the launch file available at the launch folder.

## Package Info

- **Node name:** mocker_node
- **Package name:** mocker_node

## Design

The package has one ROS2 node, which communicates via ROS2 with the car or simulator. Each node that will be mocked has a corresponding file that is responsible to 
mock the node and deliver the mock output to the ROS2 node ready to be published. The following diagram illustrates that.

![MockerNode Diagram](../../docs/assets/mocker_node/mocker_node.drawio.svg)

## Compiling
From src folder:
```sh
colcon build --packages-select mocker_node fs_msgs custom_interfaces
```

## Running the node
```sh
ros2 run mocker_node mocker_node
```
### Launch configuration

```sh
ros2 launch mocker_node mocker_node.launch.py 'sim:=eufs' 'track_name:=small_track'
```

## Main External Libraries

- [ROS2](https://docs.ros.org/en/foxy/index.html)