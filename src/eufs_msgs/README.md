# eufs_msgs

A collection of all ROS messages and services used by EUFS.

## Messages:

| Name                        | Description                                                                                                                                                                                             |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| CarState.msg                | describes the internal state of the car. Similar to the `nav_msgs/Odometry` message this contains the position, orientation, linear velocity, angular velocity, wheel speeds and slip ratio of the car. |
| CanState.msg                | used to communicate with the ADS-DV board computer via CAN. Used in `ros_can` package                                                                                                                   |
| ConeArray.msg               | used to publish 2D locations of blue, yellow, orange and big cones                                                                                                                                      |
| ConeArrayWithCovariance.msg | as ConeArray, but contains covariance info                                                                                                                                                              |
| ConeAssociation.msg | a link between two cones in the same frame, observed cone and cone in map typically |
| ConeAssociationArray.msg | a set of cone associations, matched and unmatched with metadata |
| ConeAssociationArrayArrayStamped.msg | header and array of ConeAssociationArray |
| ConeAssociationArrayStamped.msg | a set of cone associations, header and ConeAssociationArray associations |
| ConeWithCovariance.msg      | a pose and covariance matrix representing a single cone                                                                                                                                                 |
| FullState.msg               | used for data collection during dynamics learning for MPPI training                                                                                                                                     |
| IntegrationErr.msg          | Error of Odometry Integration based on euclidean distance. Used by localization's `integration_evaluator` script                                                                                        |
| LapStats.msg                | useful statistics from lap timings                                                                                                                                                                      |
| PathIntegralParams.msg      | MPPI paramteres                                                                                                                                                                                         |
| PathIntegralStats.msg       | Combines LapStats and PathIntegralParams. Used for evaluating performance of MPPI                                                                                                                       |
| PointArray.msg              | array of geometry_mgsgs/Point. Used in perception for colourless cone detections                                                                                                                        |
| PointArrayStamped.msg       | as above but with std_msgs/Header                                                                                                                                                                       |
| SLAMState.msg               | Status output message of the SLAM algorithm. Also counts laps!                                                                                                                                          |
| EKFState.msg                | Status output message of the EKF algorithm.                                                                                                                                                             |
| SLAMErr.msg                 | Error measure for SLAM, used to objectively evaluate simulation performance                                                                                                                             |
| EKFErr.msg                  | Error measure for EKF, used to objectively evaluate simulation performance                                                                                                                              |
| Particle.msg | the state of a single particle. Pose, associated map and weighting |
| ParticleSLAM.msg | the state of a particle-based SLAM algorithm. Array of Particle and index of best guess in this array |
| ParticleSLAMStamped.msg | ParticleSLAM with reference frame and timestamp. (State and header.) |
| ParticleStamped.msg | Particle with reference coordinate frame and timestamp |
| SystemState.msg             | overall mission status of the car. Used in the `ros_can` package                                                                                                                                        |
| StereoImage.msg             | Contains color and depth image                                                                                                                                                                          |
| WheelOdometryErr.msg        | Error of Wheel Odometry based on euclidean distance. Used by localization's `wheel_odometery_evaluator` script                                                                                          |
| WheelSpeeds.msg             | output of the wheel odometry. Used in the `ros_can` package                                                                                                                                             |
| WheelSpeedsStamped.msg      | same as the above but with std_msgs/Header                                                                                                                                                              |
| Waypoint.msg                | suggested control outputs at each point of path produced by planning nodes                                                                                                                              |
| WaypointArrayStamped.msg    | array of Waypoint.msg with a header                                                                                                                                                                     |

## Services:
