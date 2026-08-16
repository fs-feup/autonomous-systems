from rclpy.node import Node
from std_msgs.msg import Float64

from .stats import RunningStats, publish_float


class _SmoothnessSignal:
    """Tracks and publishes smoothness for a single scalar control signal."""

    def __init__(self, node: Node, prefix: str, name: str) -> None:
        self._stats = RunningStats()
        self._last: float | None = None
        self._current: float | None = None
        self._updated = False

        p = f"{prefix}/{name}"
        self._instant_pub = node.create_publisher(Float64, f"{p}/instant", 10)
        self._average_pub = node.create_publisher(Float64, f"{p}/average", 10)
        self._mse_pub = node.create_publisher(Float64, f"{p}/mse", 10)

    def update(self, value: float) -> None:
        if self._last is None:
            self._last = value
        else:
            self._updated = True
        self._current = value

    def publish(self) -> None:
        if not self._updated:
            return
        delta = self._current - self._last
        self._last = self._current
        self._updated = False
        self._stats.add(delta)
        publish_float(self._instant_pub, delta)
        publish_float(self._average_pub, self._stats.average_abs())
        publish_float(self._mse_pub, self._stats.mse())


class SmoothnessMetrics:
    """
    Smoothness metrics for control signals.

    To add a new signal: create a _SmoothnessSignal, add an update_* method,
    and call .publish() inside publish_metrics().
    """

    def __init__(self, node: Node, topic_prefix: str) -> None:
        p = topic_prefix
        self._left_throttle = _SmoothnessSignal(node, p, "left_throttle_smoothness")
        self._right_throttle = _SmoothnessSignal(node, p, "right_throttle_smoothness")
        self._steering = _SmoothnessSignal(node, p, "steering_smoothness")

    def update_throttle(self, left: float, right: float) -> None:
        self._left_throttle.update(left)
        self._right_throttle.update(right)

    def update_steering(self, steering: float) -> None:
        self._steering.update(steering)

    def publish_metrics(self) -> None:
        self._left_throttle.publish()
        self._right_throttle.publish()
        self._steering.publish()
