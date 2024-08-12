# MockerNode package

## Package Information

### Description

This package is used to mock outputs from other packages. It has a central ROS2 node that deals with ROS2 communication and will have 4 additional files that 
separately mock each of the following subsystems: Perception, State Estimation, Path Planning and Driverless Control. The configuration should be done by launching
with the launch file available at the launch folder.

### Launch Configurations

- [eufs.launch.py](launch/eufs.launch.py): Launches the mocker node with the configuration for EUFS simulator.
- [pacsim.launch.py](launch/pacsim.launch.py): Launches the mocker node with the configuration for PacSim simulator.

## How to Run

### Install Dependencies

```sh
./dependencies_install.sh
```

### Compiling

```sh
colcon build --packages-up-to mocker_node
```

### Running

Use a launch file:

```sh
source ./install/setup.bash # If in a new terminal
ros2 launch mocker_node eufs.launch.py
```

or run directly:

```sh
source ./install/setup.bash # If in a new terminal
ros2 run mocker_node mocker_node
```

## Design

The package has one ROS2 node, which communicates via ROS2 with the car or simulator. Each node that will be mocked has a corresponding file that is responsible to 
mock the node and deliver the mock output to the ROS2 node ready to be published. The following diagram illustrates that.

![MockerNode Diagram](../../docs/assets/mocker_node/mocker_node.drawio.svg)
