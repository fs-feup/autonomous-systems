# Lateral Control Package

The Lateral Control module contains the node that executed the lateral control of the system (control the steering mechanism).

## Run the Node

### Compiling

From src folder:
```sh
colcon build --packages-select lateral_control custom_interfaces
```

## Testing

From src folder:
```sh
colcon test --packages-select lateral_control # use event-handler=console_direct+ for imediate output
```

To check test results:
```sh
colcon test-result --all --verbose
```

or 

```sh
ros2 run loc_map loc_map_test
```

### Running the node
```sh
ros2 run lateral_control lateral_control_node
```

## Design

Below, some diagrams are presented that can illustrate the structure and behaviour of the program.

### Behaviour



### Structure



## Full Documentation



## Main External Libraries

- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [ROS2](https://docs.ros.org/en/foxy/index.html)
- [Gtest](http://google.github.io/googletest/)