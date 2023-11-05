# Changelog
All notable changes to this project will be documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0] - 2022-03-27
### Added
- [FS-AI-API](https://github.com/FS-AI/FS-AI_API) submodule for communication with the IMechE ADS-DV
- AS and AMI state publisher (String and eufs_msgs/CanState)
- Wheel speeds publisher
- Vehicle command publisher
- Twist publisher
- IMU publisher
- Fix publisher
- Control command subscription
- Subscriptions for mission finished and driving flags
- Emergency break service
- Support for acceleration commands (allowing for vehicle control in 'torque mode')
- Support mechanical and engine braking
- Data validated conversion from acceleration to axle torque requests
- Request validation within operational limits of ADS-DV
- Control command timeout
