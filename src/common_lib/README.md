# Common Lib

## Package Information

### Description

This package serves as a library for definition of functions and structures that can be used by multiple nodes of the system.

### Folder Structure

Package is called **common_lib** and has no executable. 

It is composed of multiple namespaces:

- [communication](include/common_lib/communication/): implementations related to ROS messages
- [competition_logic](include/common_lib/competition_logic/): functionality related to competition details, like mission logic or colours
- [maths](include/common_lib/maths/): mathematical formulas and structures
- [sensor_data](include/common_lib/sensor_data/): structures and functions to represent and process data from sensors
- [structures](include/common_lib/structures/): generic structures from internal logic, like Position, Pose, Cone, PathPoint...
- [vehicle_dynamics](include/common_lib/vehicle_dynamics/): formulas related to the bicycle model and other vehicle related logic

### Important Dependencies

- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)


## How to Run

### Install Dependencies

```sh
  ./dependencies_install.sh
```

### Compiling

```sh
colcon build --packages-up-to common_lib
```

### Testing

```sh
colcon test --packages-select common_lib # use event-handler=console_direct+ for imediate output
```

To check test results:
```sh
colcon test-result --all --verbose
```

