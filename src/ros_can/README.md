# ros_can

This is a ROS 2 node used to communicate with the ADS-DV over the CAN bus.

### Dependencies

- All ROS dependencies found in the package manifest as usual.
- `eufs_msgs` found [here](https://gitlab.com/eufs/eufs_msgs)
- Ability to communicate through CAN

# Nodes

## ros_can

This is a node that communicates over CAN to the ADS-DV to both receive data
from its ECU but also to control it. To achieve this it uses the C library
found in the `FS-AI-API` submodule. This node can convert acceleration and
steering (radians) commands into an appropriate car torque/braking and steering
(degrees) commands and send them to the car. It is also able to indicate
mission completion to the car. The node publishes information it receives from
the wheel odometry and steering feedback from the car, information about the
internal [state machine of the vehicle](http://www.imeche.org/docs/default-source/1-oscar/formula-student/2019/fs-ai/ads-dv-software-interface-specification-v0-2.pdf?sfvrsn=2), and data from the
onboard IMU and GPS.

### Subscribers

| Topic                         | Type                                 | Purpose                                                            |
| ----------------------------- | ------------------------------------ | ------------------------------------------------------------------ |
| `/cmd`                        | ackermann_msgs/AckermannDriveStamped | Control commands to send to the car                                |
| `/ros_can/mission_flag`       | std_msgs/Bool                        | Mission flag to control the state machine of the ADS-DV car        |
| `/state_machine/driving_flag` | std_msgs/Bool                        | Flag to indicate when ros_can should start accepting /cmd commands |

### Publishers

| Topic                       | Type                                     | Purpose                                                      |
| --------------------------- | ---------------------------------------- | ------------------------------------------------------------ |
| `/ros_can/state`            | eufs_msgs/CanState                       | AS and AMI State of the ADS-DV.                              |
| `/ros_can/state_str`        | std_msgs/String                          | Same as above but in string format. Used for debugging.      |
| `/ros_can/wheel_speeds`     | eufs_msgs/WheelSpeedsStamped             | Wheel odometry and steering feedback from the car.           |
| `/ros_can/twist`            | geometry_msgs/TwistWithCovarianceStamped | Forward linear and angular velocity based on wheel speeds.   |
| `/ros_can/vehicle_commands` | eufs_msgs/VehicleCommandsStamped         | Control information sent to the car. For debugging purposes. |
| `/ros_can/imu`              | sensor_msgs/Imu                          | Data from the onboard IMU.                                   |
| `/ros_can/fix`              | sensor_msgs/NavSatFix                    | Data from the onboard GPS.                                   |

### Parameters

| Topic              | Type   | Default | Purpose                                                                                   |
| ------------------ | ------ | ------- | ----------------------------------------------------------------------------------------- |
| `can_interface`    | string | can0    | Sets the CAN interface in FS-AI library.                                                  |
| `can_debug`        | int    | 0       | Starts the FS-AI library in debug mode. Set to something other than 0 to enable           |
| `simulate_can`     | int    | 0       | Starts the FS-AI library with simulated CAN data. Set to something other than 0 to enable |
| `loop_rate`        | int    | 100     | The running frequency of the node                                                         |
| `rpm_limit`        | float  | 4000.0  | Sets the max rpm limit that the car can achieve. Use this to set upperbound on velocity   |
| `max_dec`          | float  | 10.0    | Assume maximum deceleration of the mechanical brakes.                                     |
| `engine_threshold` | float  | -5.0    | Maximum deceleration for engine braking. Anything more and we use the mechanical brakes.  |
| `cmd_timeout`      | double | 0.5     | Timeout for control commands in seconds. Engage EBS if timeout expires.                   |
| `debug_logging`    | bool   | false   | Enable debug logging.                                                                     |

### Notes

- This node follows the UK FS-AI rules as per [documentation](https://github.com/FS-AI/FS-AI_API/blob/main/Docs/ADS-DV_Software_Interface_Specification_v4.0.pdf).
- CAN communication works over the SocketCAN interface and does not need any additional setup (apart from `sudo apt install can_utils`)
- This node is heavily reliant on the FS-AI CAN interface [library](https://github.com/FS-AI/FS-AI_API). See it for updates and further information.
- This node controls the car in "torque mode" using acceleration input commands
- To run with the DDT car, ensure you run the start script in the FS-AI-API submodule.
- We negate the steering angle received from the car. This is to make positive steering a left turn.

### TODO

See issues tab on Gitlab.

# Launch files

| Filename            | Purpose                                                                             |
| ------------------- | ----------------------------------------------------------------------------------- |
| `ros_can.launch.py` | Starts the `ros_can` node in normal working operation configured for the ADS-DV car |
