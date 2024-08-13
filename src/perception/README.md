# Perception Package

## Package Information

### Description

The Perception module is responsible for the processing of the perception module and perception sensors' data and its transformation into useful information. In this case, the LiDAR's point cloud is processed to generate the cones' position on the track.

### Folder Structure

- [center_calculation](include/center_calculation/): Functions to calculate the center of the cones
- [cone_differentiation](include/cone_differentiation/): Functions to differentiate cones of different types
- [cone_validator](include/cone_validator/): Functions to validate cones
- [perception](include/perception/): Perception node class
- [clustering](include/clustering/): Functions to create clusters of cones
- [cone_evaluator](include/cone_evaluator/): Functions to evaluate cone detections' quality
- [ground_removal](include/ground_removal/): Functions to remove the ground from the point cloud
- [utils](include/utils/): Utility functions
- [icp](include/icp/): Functions to perform ICP, used in a validation method

### Launch Configurations

- [eufs.launch.py](launch/eufs.launch.py): Launch file for the EUFS simulator
- [rosbag-preprocessed.launch.py](launch/rosbag-preprocessed.launch.py): Launch file for the a preprocessed rosbag with a cone scene
- [vehicle.launch.py](launch/vehicle.launch.py): Launch file for the 01 vehicle


### Important Dependencies

- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [PCL](https://pointclouds.org/)


## How to Run

### Install Dependencies

```sh
./dependencies_install.sh
```

### Compiling

```sh
colcon build --packages-up-to perception
```

### Testing

```sh
colcon test --packages-select perception # use event-handler=console_direct+ for imediate output
```

To check test results:
```sh
colcon test-result --all --verbose
```

### Running

Use a launch file:

```sh
source ./install/setup.bash # If in a new terminal
ros2 launch perception eufs.launch.py
```

or run directly:

```sh
source ./install/setup.bash # If in a new terminal
ros2 run perception perception
```


## Design

Below, some diagrams are presented that can illustrate the structure and behaviour of the program.

### Class Diagram

![Perception Class Diagram](../../docs/diagrams/perception/class_diagram.drawio.png)
