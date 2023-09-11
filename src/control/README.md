# Control Package

## Compiling

From src folder:
```sh
colcon build --packages-select control custom_interfaces
```

## Running
```sh
ros2 run control control
```

This command will launch the control_node responsible for both the node processing and the communication management.