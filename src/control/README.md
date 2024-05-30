# Longitudinal Controller

Control Node

## Dependencies

* custom_interfaces
* pacsim
* fs_msgs
* eufs_msgs

## Compiling

From src folder:
```sh
colcon build --package-up-to control
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