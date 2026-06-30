# control_evaluator

ROS 2 Python package with a node named `control_evaluator` for control-performance evaluation.

The node reads `global.adapter` from `config/global/global_config.yaml` and switches topics per environment.

## Subscriptions

- Always:
	- `/path_planning/path` (`custom_interfaces/msg/PathPointArray`)
	- `/state_estimation/lap_counter` (`std_msgs/msg/Float64`)

- If adapter is `vehicle`:
	- `/state_estimation/pose` (`custom_interfaces/msg/Pose`)
	- `/state_estimation/velocities` (`custom_interfaces/msg/Velocities`)

- If adapter is `pacsim`:
	- `/pacsim/pose` (`geometry_msgs/msg/TwistWithCovarianceStamped`)
	- `/pacsim/velocity` (`geometry_msgs/msg/TwistWithCovarianceStamped`)

## Metrics start condition

Metrics start only after the first control command is observed.

- If adapter is `vehicle`: first message on `/control/command` (`custom_interfaces/msg/ControlCommand`)
- If adapter is `pacsim`: first message on either `/pacsim/steering_setpoint` (`pacsim/msg/StampedScalar`) or `/pacsim/throttle_setpoint` (`pacsim/msg/Wheels`)

## Published metrics (prefix: `/control_evaluator`)

For each instant, average, mean squared error, and root mean squared error:

- `distance_error/{instant,average,mse,rmse}`
- `velocity_error/{instant,average,signed_average,mse,rmse}`
- `heading_error/{instant,average,signed_average,mse,rmse}`

Notes:

- Distance is computed to the interpolated line segment between the closest path point and its neighboring segment.
- Heading and reference velocity are computed from linear interpolation on that selected segment.
- `average` for error metrics is the mean absolute error (MAE).
- `signed_average` is the signed mean (bias).

Additional metrics:

- `average_velocity`
- `average_lap_time`

## Run

```bash
colcon build --packages-select control_evaluator
source install/setup.bash
ros2 launch control_evaluator control_evaluator.launch.py
```
