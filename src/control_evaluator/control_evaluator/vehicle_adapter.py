import math

from custom_interfaces.msg import ControlCommand, Pose, Velocities

from . import node
from . import params as params_module


class VehicleAdapter(node.ControlEvaluatorNode):
    def __init__(self, params: params_module.Params) -> None:
        super().__init__(params)

        self._pose_sub = self.create_subscription(
            Pose, "/state_estimation/pose", self._vehicle_pose_callback, 10
        )
        self._vel_sub = self.create_subscription(
            Velocities, "/state_estimation/velocities", self._vehicle_velocities_callback, 10
        )
        self.create_subscription(
            ControlCommand, "/control/command", self._vehicle_control_callback, 10
        )

    def _vehicle_pose_callback(self, msg: Pose) -> None:
        self._error_metrics.update_pose(msg.x, msg.y, msg.theta)

    def _vehicle_velocities_callback(self, msg: Velocities) -> None:
        speed = math.sqrt(msg.velocity_x ** 2 + msg.velocity_y ** 2)
        self._error_metrics.update_speed(speed)
        self._performance_metrics.update_velocity(speed)

    def _vehicle_control_callback(self, msg: ControlCommand) -> None:
        left_throttle = (msg.throttle_fl + msg.throttle_rl) / 2.0
        right_throttle = (msg.throttle_fr + msg.throttle_rr) / 2.0
        self._smoothness_metrics.update_throttle(left_throttle, right_throttle)
        self._smoothness_metrics.update_steering(msg.steering)
        self._notify_control_received()
