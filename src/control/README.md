# Longitudinal Controller

Control Node

## Dependencies

N/a (at the moment)

## Compiling

From src folder:
```sh
colcon build --packages-select control custom_interfaces fs_msgs --symlink-install
```

## Testing

From src folder:
```sh
colcon test --packages-select control # use event-handler=console_direct+ for imediate output
```

## Running
```sh
ros2 run control control
```