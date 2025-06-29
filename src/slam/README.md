# SLAM Package

## Package Information


### Description

This package defines the SLAM node: a node responsible for performing localization and mapping. It receives linear and angular velocities and perception relative cone coordinates and outputs an estimate of the vehicle's pose and the map of the track.


### Folder Structure

- [adapter_slam](./include/slam/): Adapters to change ros2 interfaces according to simulator or environment
- [slam_solver](./include/slam_solver/): Includes the actual solvers of the SLAM problem
    - [slam_solver/graph_slam](./include/slam_solver/graph_slam/): Code for using smoothing based SLAM with GTSAM library
    - [slam_solver/ekf_slam](./include/slam_solver/ekf_slam/): Code for using Extended Kalman Filter based SLAM
- [ros_node](./include/ros_node/): Node class
- [slam_config](./include/slam_config/): Configuration files for the SLAM node with parameters for the solvers


### Launch Configurations

- [vehicle.launch.py](./launch/vehicle.launch.py): Launch file for the vehicle environment
- [pacsim.launch.py](./launch/pacsim.launch.py): Launch file for the PacSim simulator

### Important Dependencies

- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [GTSAM](https://gtsam.org/)

## How to Run

### Install Dependencies

```sh
  ./dependencies_install.sh
```

### Compiling

```sh
colcon build --packages-up-to slam
```

### Testing

```sh
colcon test --packages-select slam # use event-handler=console_direct+ for imediate output
```

To check test results:
```sh
colcon test-result --all --verbose
```

or 

```sh
source ./install/setup.bash # If in a new terminal
ros2 run slam slam_test
```

### Running

Use a launch file:

```sh
source ./install/setup.bash # If in a new terminal
ros2 launch slam eufs.launch.py
```

or run directly:


```sh
source ./install/setup.bash # If in a new terminal
ros2 run slam slam
```

## Design

Below, some diagrams are presented that can illustrate the structure and behaviour of the program.

### Behaviour


### Structure

The node is composed by multiple classes. The diagram below illustrates roughly how they sit in the code structure.

![Class Diagram](../../docs/diagrams/slam/class.drawio.svg)


The following node shows the interfaces of this node.
![Components Diagram](../../docs/diagrams/slam/components.drawio.svg)