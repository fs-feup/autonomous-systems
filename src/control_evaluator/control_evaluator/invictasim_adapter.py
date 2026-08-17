import math

from custom_interfaces.msg import ControlCommand, Pose, Velocities

from . import node
from . import params as params_module


class InvictasimAdapter(node.ControlEvaluatorNode):
    def __init__(self, params: params_module.Params) -> None:
        super().__init__(params)

        if params.using_simulated_slam:
            self._pose_sub = self.create_subscription(
                Pose,
                "/invictasim/state_estimation/vehicle_pose",
                self._vehicle_pose_callback,
                10,
            )
        if params.using_simulated_se:
            self._vel_sub = self.create_subscription(
                Velocities,
                "/invictasim/state_estimation/velocities",
                self._vehicle_velocities_callback,
                10,
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
        self._smoothness_metrics.update_throttle(msg.throttle_rl, msg.throttle_rr)
        self._smoothness_metrics.update_steering(msg.steering)
        self._notify_control_received()
