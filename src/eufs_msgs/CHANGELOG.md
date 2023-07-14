# Changelog
All notable changes to this project will be documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0] - 2022-01-05
### Added
- IntegrationErr msg
- WheelOdometryErr msg
- AMI_JOYSTICK in CanState msg (AMI_MANUAL moved to 21)
- Speed variance, normalized path deviation MSE, path deviation variance and max path deviation in LapStats msg
- Support frame percentage in BoundingBox msg
### Changed
- Remove duplication of WheelSpeeds attributes in WheelSpeedsStamped msg
### Removed
- Mission flag in CanState msg

## [1.0.0] - 2022-01-05
### Added
- BoundingBox msg
- BoundingBoxes msg
- CanState msg
- CarState msg
- ChassisCommand msg
- ChassisState msg
- ConeArray msg
- ConeArrayWithCovariance msg
- ConeWithCovariance msg
- Costmap msg
- EKFErr msg
- EKFState msg
- FullState msg
- LapStats msg
- PathIntegralParams msg
- PathIntegralStats msg
- PathIntegralStatus msg
- PathIntegralTiming msg
- PointArray msg
- PointArrayStamped msg
- Runstop msg
- SLAMErr msg
- SLAMState msg
- SystemStatus msg
- TopicStatus msg
- Waypoint msg
- WaypointArrayStamped msg
- WheelSpeeds msg
- WheelSpeedsStamped msg
- CheckForObjects action

