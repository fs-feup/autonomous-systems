# Change Log

All notable changes to this project will be documented in this file.

## Sprint 8

### Localization and Mapping
- Full EKF SLAM implemented, without noise matrixes

## Sprint 2

### Localization and Mapping
- Localization and Mapping publisher node
- Cone, ConeArray and Pose messages

### Control
- Interface setup for control/path-planning exchange
- Communication with simulator, although not with the right path information

## Sprint 1

### Control

- The pre-existing PID algorithm was integrated into a ROS Node
- A path-planning mock publisher was created so that the control node could receive test messages
- A custom interfaces package was built to support any custom messages in the project
