# Emergency Remote Control

ROS2 websocket receiver for the Expo emergency intervention app.

## Websocket Payload

The node accepts JSON websocket messages with this shape:

```json
{
  "type": "emergency_control",
  "steering_rad": 0.0,
  "throttle": 0.0,
  "e_ebs": false,
  "take_control": false,
  "sent_at_ms": 1234567890
}
```

`steering_rad` is clamped to `[-0.335, 0.335]`. `throttle` is clamped to `[-1.0, 1.0]`.

## ROS Interfaces

Published:

- `/remote/ebs` (`std_msgs/msg/Bool`)
- `/remote/take_control` (`std_msgs/msg/Bool`)
- `/control/command` (`custom_interfaces/msg/ControlCommand`)

Subscribed:

- `/vehicle/motor_rpm` (`custom_interfaces/msg/WheelRPM`)

The existing `control` node also subscribes to `/remote/ebs` and `/remote/take_control`. It stops publishing autonomous `/control/command` messages while either remote EBS or remote manual control is active.

## Behavior

- `e_ebs=true` has priority over `take_control`.
- Every received websocket payload is printed to the terminal before validation.
- During EBS, app throttle is ignored. The node runs a PID controller against `/vehicle/motor_rpm` and publishes rear-wheel brake/throttle commands until motor RPM is inside `ebs_stop_rpm_tolerance`.
- EBS PID error is clamped between `velocity_error_min` and `velocity_error_max`. The defaults are `-2000` and `2000` RPM.
- During manual remote control, the node publishes app steering and rear-wheel throttle to `/control/command`.
- When neither EBS nor take-control is active, the node only publishes `/remote/ebs` and `/remote/take_control`.
- If messages become stale or the websocket disconnects, remote authority is not released automatically.
- Remote manual control remains active until a fresh payload explicitly sets `take_control=false`.
- EBS remains latched until a fresh payload explicitly sets `e_ebs=false`.
- The websocket server accepts multiple simultaneous clients. If the phone connection dies, a new phone connection can be accepted without restarting this node.

## Parameters

- `websocket_host`: bind address, default `0.0.0.0`
- `websocket_port`: bind port, default `4321`
- `websocket_idle_timeout_ms`: optional dead websocket connection timeout, default `0` disabled
- `stale_timeout_ms`: stale-message warning/log timeout, default `500`
- `command_period_ms`: command loop period, default `25`
- `steering_min_rad`, `steering_max_rad`: steering clamp
- `throttle_min`, `throttle_max`: throttle clamp
- `velocity_error_min`, `velocity_error_max`: RPM error clamp used by the EBS PID, default `-2000.0` and `2000.0`
- `ebs_stop_rpm_tolerance`: motor RPM considered stopped, default `50.0`
- `ebs_pid_kp`, `ebs_pid_ki`, `ebs_pid_kd`: EBS PID gains
- `ebs_pid_output_min`, `ebs_pid_output_max`: PID output limits
- `ebs_pid_integral_min`, `ebs_pid_integral_max`: PID integral limits

Default EBS scaling:

```text
target control / max RPM error = 0.4 / 2000 = 0.0002
```

So `ebs_pid_kp=0.0002`, `velocity_error_min=-2000`, `velocity_error_max=2000`, and `ebs_pid_output_min/max=-0.4/0.4`.

## Run

Build with the workspace script:

```sh
./compile.sh
```

Run only the remote receiver:

```sh
source install/setup.bash
ros2 launch emergency_remote_control remote_control.launch.py
```

Run through the existing control launch:

```sh
source install/setup.bash
ros2 launch control control.launch.py
```

Send a test payload:

```sh
python3 - <<'PY'
import asyncio, json, websockets

async def main():
    async with websockets.connect("ws://127.0.0.1:4321") as ws:
        await ws.send(json.dumps({
            "type": "emergency_control",
            "steering_rad": 0.1,
            "throttle": 0.2,
            "e_ebs": False,
            "take_control": True,
            "sent_at_ms": 0,
        }))

asyncio.run(main())
PY
```

Run package tests:

```sh
colcon test --packages-select emergency_remote_control --event-handlers console_direct+
colcon test-result --all --verbose
```
