import math

from custom_interfaces.msg import PathPointArray, Pose, Velocities
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from . import params as params_module
from .error_metrics import ErrorMetrics
from .performance_metrics import PerformanceMetrics
from .smoothness_metrics import SmoothnessMetrics


class ControlEvaluatorNode(Node):
    def __init__(self, params: params_module.Params) -> None:
        super().__init__("control_evaluator")
        self._params = params
        self.get_logger().info(f"control_evaluator adapter: {self._params.adapter}")

        prefix = "/control_evaluator"
        self._error_metrics = ErrorMetrics(self, prefix)
        self._performance_metrics = PerformanceMetrics(self, prefix)
        self._smoothness_metrics = SmoothnessMetrics(self, prefix)

        self._path_sub = self.create_subscription(
            PathPointArray, "/path_planning/path", self._path_callback, 10
        )
        self._lap_counter_sub = self.create_subscription(
            Float64, "/state_estimation/lap_counter", self._lap_counter_callback, 10
        )

        if params.using_simulated_slam:
            self._pose_sub = self.create_subscription(
                Pose, "/state_estimation/pose", self._default_pose_callback, 10
            )
        if params.using_simulated_se:
            self._vel_sub = self.create_subscription(
                Velocities, "/state_estimation/velocities", self._default_velocity_callback, 10
            )

        self.create_timer(0.05, self._publish_metrics)
        self.get_logger().info("control_evaluator node started")

    def _path_callback(self, msg: PathPointArray) -> None:
        path_points = [(p.x, p.y, p.v, p.orientation) for p in msg.pathpoint_array]
        self._error_metrics.update_path(path_points)

    def _lap_counter_callback(self, msg: Float64) -> None:
        self._performance_metrics.notify_lap()

    def _default_pose_callback(self, msg: Pose) -> None:
        self._error_metrics.update_pose(msg.x, msg.y, msg.theta)

    def _default_velocity_callback(self, msg: Velocities) -> None:
        speed = math.sqrt(msg.velocity_x ** 2 + msg.velocity_y ** 2)
        self._error_metrics.update_speed(speed)
        self._performance_metrics.update_velocity(speed)

    def _notify_control_received(self) -> None:
        self._error_metrics.notify_control_received()
        self._performance_metrics.notify_control_received()

    def _publish_metrics(self) -> None:
        self._error_metrics.publish_metrics()
        self._performance_metrics.publish_metrics()
        self._smoothness_metrics.publish_metrics()


def main(args=None) -> None:
    from . import vehicle_adapter, pacsim_adapter, invictasim_adapter
    rclpy.init(args=args)
    params = params_module.Params()
    node = None
    if params.adapter == "vehicle":
        node = vehicle_adapter.VehicleAdapter(params)
    elif params.adapter == "pacsim":
        node = pacsim_adapter.PacsimAdapter(params)
    elif params.adapter == "invictasim":
        node = invictasim_adapter.InvictasimAdapter(params)
    else:
        raise ValueError(f"Unknown adapter: {params.adapter}")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
