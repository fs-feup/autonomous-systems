from . import node
import math

from geometry_msgs.msg import TwistWithCovarianceStamped
from pacsim.msg import StampedScalar, Wheels
from . import params as params_module

class PacsimAdapter(node.ControlEvaluatorNode):
    def __init__(self, params: params_module.Params) -> None:
        super().__init__(params)

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
        self._pacsim_steering_control_sub = self.create_subscription(
            StampedScalar,
            "/pacsim/steering_setpoint",
            self._pacsim_steering_callback,
            10,
        )
        self._pacsim_throttle_control_sub = self.create_subscription(
            Wheels,
            "/pacsim/throttle_setpoint",
            self._pacsim_throttle_callback,
            10,
        )
        self.has_received_steering_command = False
        self.has_received_throttle_command = False

    def _pacsim_steering_callback(self, msg: StampedScalar) -> None:
        self._update_steering_smoothness(msg.value)
        self.has_received_steering_command = True
        if self.has_received_throttle_command:
            self._has_received_control_command = True

    def _pacsim_throttle_callback(self, msg: Wheels) -> None:
        left_throttle = (msg.fl + msg.rl) / 2.0
        right_throttle = (msg.fr + msg.rr) / 2.0
        self._update_throttle_smoothness(left_throttle, right_throttle)
        self.has_received_throttle_command = True
        if self.has_received_steering_command:
            self._has_received_control_command = True

    def _pacsim_velocity_callback(self, msg: TwistWithCovarianceStamped) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._current_speed = math.sqrt(vx * vx + vy * vy)
        self._has_speed = True
        if self._has_received_control_command:
            self._average_velocity_stats.add(self._current_speed)
            self._publish_float(
                self._average_velocity_pub, self._average_velocity_stats.average()
            )

    def _pacsim_pose_callback(self, msg: TwistWithCovarianceStamped) -> None:
        self._current_x = msg.twist.twist.linear.x
        self._current_y = msg.twist.twist.linear.y
        self._current_heading = msg.twist.twist.angular.z
        self._has_pose = True