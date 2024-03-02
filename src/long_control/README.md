# Longitudinal Controller

Control Node

## Dependencies

N/a (at the moment)

## Compiling

From src folder:
```sh
colcon build --packages-select long_control custom_interfaces fs_msgs
```

## Testing

From src folder:
```sh
colcon test --packages-select long_control # use event-handler=console_direct+ for imediate output
```

## Running
```sh
ros2 run long_control long_control
```