from . import node
import math

from custom_interfaces.msg import ControlCommand, PathPointArray, Pose, Velocities
from . import params as params_module

class VehicleAdapter(node.ControlEvaluatorNode):
    def __init__(self, params: params_module.Params) -> None:
        super().__init__(params)

        self._pose_sub = self.create_subscription(
            Pose,
            "/state_estimation/pose",
            self._vehicle_pose_callback,
            10,
        )
        self._vel_sub = self.create_subscription(
            Velocities,
            "/state_estimation/velocities",
            self._vehicle_velocities_callback,
            10,
        )
        self._control_sub = self.create_subscription(
            ControlCommand,
            "/control/command",
            self._vehicle_control_callback,
            10,
        )

    def _vehicle_pose_callback(self, msg: Pose) -> None:
        self._current_x = msg.x
        self._current_y = msg.y
        self._current_heading = msg.theta
        self._has_pose = True

    def _vehicle_velocities_callback(self, msg: Velocities) -> None:
        self._current_speed = math.sqrt(
            msg.velocity_x * msg.velocity_x + msg.velocity_y * msg.velocity_y
        )
        self._has_speed = True
        if self._has_received_control_command:
            self._average_velocity_stats.add(self._current_speed)
            self._publish_float(
                self._average_velocity_pub, self._average_velocity_stats.average()
            )

    def _vehicle_control_callback(self, msg: ControlCommand) -> None:
        left_throttle = (msg.throttle_fl + msg.throttle_rl) / 2.0
        right_throttle = (msg.throttle_fr + msg.throttle_rr) / 2.0
        self._update_throttle_smoothness(left_throttle, right_throttle)
        self._update_steering_smoothness(msg.steering)
        self._has_received_control_command = True