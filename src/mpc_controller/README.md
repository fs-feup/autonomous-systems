# MPC Controller

Acados-based lateral MPC with a longitudinal PID to follow the path planning trajectory and velocity profile. The node reuses the existing interfaces:

- Publishes `custom_interfaces/ControlCommand` to `/control/command` (vehicle adapter)
- Publishes `pacsim/StampedScalar` to `/pacsim/steering_setpoint` and `/pacsim/throttle_setpoint` (pacsim adapter)
- Publishes `ackermann_msgs/AckermannDriveStamped` to `/cmd` (EUFS)

## Prerequisites

- `acados` installed (run `./install_acados.sh`, sets up libs and Python templates).
- `casadi`, `acados_template`, and `numpy` available in the active Python environment (handled by the install script above).

## Build & Run

```bash
colcon build --packages-up-to mpc_controller
source install/setup.bash
ros2 launch mpc_controller mpc_controller.launch.py
```

The node reads:

- Global config: `config/global/global_config.yaml` to pick adapter and simulated inputs.
- Control config: `config/control/<adapter>.yaml` for PID and MPC weights/horizon (`mpc_*` keys).
- Vehicle parameters: `config/car/<adapter>.yaml` for dynamic bicycle parameters.

Tune MPC weights (`mpc_w_*`), horizon, dt, and steering limits in the control config. Steering bounds default to values from the car YAML if present; otherwise, they fall back to the control config.
