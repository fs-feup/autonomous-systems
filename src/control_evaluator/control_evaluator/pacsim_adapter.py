import math

from geometry_msgs.msg import TwistWithCovarianceStamped
from pacsim.msg import StampedScalar, Wheels

from . import node
from . import params as params_module


class PacsimAdapter(node.ControlEvaluatorNode):
    def __init__(self, params: params_module.Params) -> None:
        super().__init__(params)
        self._has_steering = False
        self._has_throttle = False

        if params.using_simulated_slam:
            self._pose_sub = self.create_subscription(
                TwistWithCovarianceStamped,
                "/pacsim/pose",
                self._pacsim_pose_callback,
                10,
            )
        if params.using_simulated_se:
            self._vel_sub = self.create_subscription(
                TwistWithCovarianceStamped,
                "/pacsim/velocity",
                self._pacsim_velocity_callback,
                10,
            )

        self.create_subscription(
            StampedScalar, "/pacsim/steering_setpoint", self._pacsim_steering_callback, 10
        )
        self.create_subscription(
            Wheels, "/pacsim/throttle_setpoint", self._pacsim_throttle_callback, 10
        )

    def _pacsim_steering_callback(self, msg: StampedScalar) -> None:
        self._smoothness_metrics.update_steering(msg.value)
        self._has_steering = True
        if self._has_throttle:
            self._notify_control_received()

    def _pacsim_throttle_callback(self, msg: Wheels) -> None:
        self._smoothness_metrics.update_throttle(msg.rl, msg.rr)
        self._has_throttle = True
        if self._has_steering:
            self._notify_control_received()

    def _pacsim_velocity_callback(self, msg: TwistWithCovarianceStamped) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx * vx + vy * vy)
        self._error_metrics.update_speed(speed)
        self._performance_metrics.update_velocity(speed)

    def _pacsim_pose_callback(self, msg: TwistWithCovarianceStamped) -> None:
        self._error_metrics.update_pose(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        )
