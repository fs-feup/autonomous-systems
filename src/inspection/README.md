# Inspection package

## Package Information

### Description

This package is responsible for running two scripts used for testing the car. One of them performs the autonomous mission described in the rules: "T14.11.2 The inspection mission is defined by slowly spinning the drivetrain and actuating the steering system with a sine wave while the vehicle is jacked up and all wheels are removed. After 25 s to 30 s the AS must transition to 'AS Finished'". The other script is an implementation of the EBS test described in the rules. The parameters to be used when running this node can be configured by changing the config.txt file.

### Folder Structure

- [functions](include/functions/): Functions used in the inspection node to control the vehicle
- [inspection](include/inspection/): Inspection node class

### Launch Configurations

- [normal.launch.py](launch/normal.launch.py): Launch file for the inspection node

### Important Dependencies


## How to Run

### Compiling
```sh
colcon build --packages-up-to inspection
```

### Testing
```sh
colcon test --packages-select inspection # use event-handler=console_direct+ for imediate output
```

To check test results:
```sh
colcon test-result --all --verbose
```

### Running

To run the inspection node, use the following command:

```sh
source ./install/setup.bash # If in a new terminal
ros2 launch inspection normal.launch.py
```

or less recommended:

```sh
source ./install/setup.bash # If in a new terminal
ros2 run inspection inspection
```
## Design

The package has two classes, one of them being a ROS2 node, and responsible for all the ROS2 communication. The other class has all the functions and variables related to the package's roles.
![Inspection Diagram Part 3](../../docs/assets/Inspection/inspection.drawio.svg)
