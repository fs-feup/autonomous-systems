# Inspection package

This package is responsible for running two scripts used for testing the car. One of them performs the autonomous mission described in the rules: "T14.11.2 The inspection mission is defined by slowly spinning the drivetrain and actuating the steering system with a sine wave while the vehicle is jacked up and all wheels are removed. After 25 s to 30 s the AS must transition to 'AS Finished'". The other script is an implementation of the EBS test described in the rules. The parameters to be used when running this node can be configured by changing the config.txt file.

## Compiling
From src folder:
```sh
colcon build --packages-select inspection fs_msgs custom_interfaces
```

## Testing
From src folder:
```sh
colcon test --packages-select inspection # use event-handler=console_direct+ for imediate output
```

## Running the node
```sh
ros2 run inspection inspection
```
## Design
The package has two classes, one of them being a ROS2 node, and responsible for all the ROS2 communication. The other class has all the functions and vara+iables related to the package's roles.
![Inspection Diagram Part 3](../../docs/assets/Inspection/inspection.drawio.svg)
## Main External Libraries

- [ROS2](https://docs.ros.org/en/foxy/index.html)
- [Gtest](http://google.github.io/googletest/)