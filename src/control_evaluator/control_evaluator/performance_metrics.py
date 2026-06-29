from rclpy.node import Node
from std_msgs.msg import Float64

from .stats import RunningStats, publish_float


class PerformanceMetrics:
    """
    Performance metrics (lap time, average velocity).

    To add a new metric: add a RunningStats tracker, an update_* method
    to feed it, and a publish call inside publish_metrics().
    """

    def __init__(self, node: Node, topic_prefix: str) -> None:
        self._node = node
        p = topic_prefix

        self._lap_time_pub = node.create_publisher(Float64, f"{p}/lap_time", 10)
        self._avg_lap_time_pub = node.create_publisher(Float64, f"{p}/average_lap_time", 10)
        self._avg_velocity_pub = node.create_publisher(Float64, f"{p}/average_velocity", 10)

        self._lap_time_stats = RunningStats()
        self._velocity_stats = RunningStats()

        self._has_control = False
        self._last_lap_stamp = None
        self._last_lap_time: float | None = None

    def notify_control_received(self) -> None:
        self._has_control = True

    def update_velocity(self, speed: float) -> None:
        if not self._has_control:
            return
        self._velocity_stats.add(speed)

    def notify_lap(self) -> None:
        if not self._has_control:
            return
        now = self._node.get_clock().now()
        if self._last_lap_stamp is None:
            self._last_lap_stamp = now
            return
        lap_time = (now - self._last_lap_stamp).nanoseconds * 1e-9
        if lap_time > 0.0:
            self._lap_time_stats.add(lap_time)
            self._last_lap_time = lap_time
        self._last_lap_stamp = now

    def publish_metrics(self) -> None:
        if self._velocity_stats.count > 0:
            publish_float(self._avg_velocity_pub, self._velocity_stats.average())
        if self._last_lap_time is not None:
            publish_float(self._lap_time_pub, self._last_lap_time)
            publish_float(self._avg_lap_time_pub, self._lap_time_stats.average())
