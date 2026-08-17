from rclpy.node import Node
from std_msgs.msg import Float64

from .stats import RunningStats, publish_float
from . import reference


class _ErrorSignal:
    """Tracks and publishes a single signed or unsigned error metric."""

    def __init__(self, node: Node, prefix: str, name: str, signed: bool) -> None:
        self._stats = RunningStats()
        self._signed = signed

        p = f"{prefix}/{name}"
        self._instant_pub = node.create_publisher(Float64, f"{p}/instant", 10)
        self._average_pub = node.create_publisher(Float64, f"{p}/average", 10)
        self._mse_pub = node.create_publisher(Float64, f"{p}/mse", 10)
        if signed:
            self._abs_average_pub = node.create_publisher(Float64, f"{p}/absolute_average", 10)

    def record(self, value: float) -> None:
        self._stats.add(value)
        publish_float(self._instant_pub, value)
        if self._signed:
            publish_float(self._average_pub, self._stats.average())
            publish_float(self._abs_average_pub, self._stats.average_abs())
        else:
            publish_float(self._average_pub, self._stats.average_abs())
        publish_float(self._mse_pub, self._stats.mse())


class ErrorMetrics:
    """
    Path-tracking error metrics (distance, heading, velocity).

    To add a new error: create an _ErrorSignal and call .record() inside publish_metrics().
    State must be kept up to date via the update_* methods before publish_metrics() is called.
    """

    def __init__(self, node: Node, topic_prefix: str) -> None:
        p = topic_prefix
        self._distance = _ErrorSignal(node, p, "distance_error", signed=False)
        self._heading = _ErrorSignal(node, p, "heading_error", signed=True)
        self._velocity = _ErrorSignal(node, p, "velocity_error", signed=True)

        self._has_control = False
        self._has_pose = False
        self._has_path = False
        self._has_speed = False

        self._x = self._y = self._heading_val = self._speed = 0.0
        self._path_points: list = []
        self._cached_ref = None

    def notify_control_received(self) -> None:
        self._has_control = True

    def update_pose(self, x: float, y: float, heading: float) -> None:
        self._x, self._y, self._heading_val = x, y, heading
        self._has_pose = True
        self._cached_ref = None

    def update_path(self, path_points: list) -> None:
        self._path_points = path_points
        self._has_path = True
        self._cached_ref = None

    def update_speed(self, speed: float) -> None:
        self._speed = speed
        self._has_speed = True

    def publish_metrics(self) -> None:
        if not (self._has_control and self._has_pose and self._has_path):
            return
        ref = self._get_reference()
        self._distance.record(ref[0])
        self._heading.record(reference.normalize_angle(self._heading_val - ref[1]))
        if self._has_speed:
            self._velocity.record(self._speed - ref[2])

    def _get_reference(self):
        if self._cached_ref is None:
            self._cached_ref = reference.compute_path_reference(
                self._x, self._y, self._path_points
            )
        return self._cached_ref
