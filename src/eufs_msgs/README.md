# eufs_msgs
A collection of all ROS messages and services used by the team

## Messages:
| Name | Description |
| ---- | ---- |
| CarState.msg | describes the internal state of the car. Similar to the `nav_msgs/Odometry` message this contains the position, orientation, linear velocity, angular velocity, wheel speeds and slip ratio of the car. |
| CanState.msg | used to communicate with the ADS-DV board computer via CAN. Used in `ros_can` package |
| ConeArray.msg | used to publish 2D locations of blue, yellow, orange and big cones |
| ConeArrayWithCovariance.msg | as ConeArray, but contains covariance info |
| ConeWithCovariance.msg | a pose and covariance matrix representing a single cone |
| FullState.msg | used for data collection during dynamics learning for MPPI training |
| IntegrationErr.msg | Error of Odometry Integration based on euclidean distance. Used by localization's `integration_evaluator` script |
| LapStats.msg | useful statistics from lap timings |
| PathIntegralParams.msg | MPPI paramteres |
| PathIntegralStats.msg |Combines LapStats and PathIntegralParams. Used for evaluating performance of MPPI |
| PointArray.msg | array of geometry_mgsgs/Point. Used in perception for colourless cone detections |
| PointArrayStamped.msg | as above but with std_msgs/Header |
| SLAMState.msg | Status output message of the SLAM algorithm. Also counts laps! |
| EKFState.msg | Status output message of the EKF algorithm. |
| SLAMErr.msg | Error measure for SLAM, used to objectively evaluate simulation performance |
| EKFErr.msg | Error measure for EKF, used to objectively evaluate simulation performance |
| SystemState.msg | overall mission status of the car. Used in the `ros_can` package |
| WheelOdometryErr.msg | Error of Wheel Odometry based on euclidean distance. Used by localization's `wheel_odometery_evaluator` script |
| WheelSpeeds.msg | output of the wheel odometry. Used in the `ros_can` package |
| WheelSpeedsStamped.msg | same as the above but with std_msgs/Header |
| Waypoint.msg | suggested control outputs at each point of path produced by planning nodes
| WaypointArrayStamped.msg | array of Waypoint.msg with a header
## Services:

