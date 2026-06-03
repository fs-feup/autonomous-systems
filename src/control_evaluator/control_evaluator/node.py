import math
from typing import Optional

import rclpy
from custom_interfaces.msg import ControlCommand, PathPointArray, Pose, Velocities

from rclpy.node import Node
from std_msgs.msg import Float64

from . import params as params_module

class RunningStats:
    def __init__(self) -> None:
        self.count = 0
        self.sum = 0.0
        self.abs_sum = 0.0
        self.sum_sq = 0.0

    def add(self, value: float) -> None:
        self.count += 1
        self.sum += value
        self.abs_sum += abs(value)
        self.sum_sq += value * value

    def average(self) -> float:
        if self.count == 0:
            return 0.0
        return self.sum / self.count

    def average_abs(self) -> float:
        if self.count == 0:
            return 0.0
        return self.abs_sum / self.count

    def mse(self) -> float:
        if self.count == 0:
            return 0.0
        return self.sum_sq / self.count

class ControlEvaluatorNode(Node):
    def __init__(self, params: params_module.Params) -> None:
        super().__init__("control_evaluator")

        self._params = params

        self.get_logger().info(f"control_evaluator adapter: {self._params.adapter}")

        self.topic_prefix = "/control_evaluator"

        # Metric definitions: metric name -> (has_sign, can_compute_fn, compute_fn)
        self.metric_has_sign = {
            "distance_error": (
                False,
                lambda: self._has_received_control_command and self._has_received_path and self._has_pose,
                lambda: self._get_path_reference()[0],
            ),
            "velocity_error": (
                True,
                lambda: self._has_received_control_command and self._has_received_path and self._has_pose and self._has_speed,
                lambda: self._current_speed - self._get_path_reference()[2],
            ),
            "heading_error": (
                True,
                lambda: self._has_received_control_command and self._has_received_path and self._has_pose,
                lambda: self._normalize_angle(self._current_heading - self._get_path_reference()[1]),
            ),
            "left_throttle_smoothness": (
                False,
                lambda: self._left_throttle_updated,
                self._compute_left_throttle_smoothness,
            ),
            "right_throttle_smoothness": (
                False,
                lambda: self._right_throttle_updated,
                self._compute_right_throttle_smoothness,
            ),
            "steering_smoothness": (
                False,
                lambda: self._steering_updated,
                self._compute_steering_smoothness,
            ),
        }

        # Error publisher creation
        for metric, metric_info in self.metric_has_sign.items():
            has_sign = metric_info[0]
            for topic in ("instant", "average", "mse"):
                setattr(self, f"_{metric}_{topic}_pub",
                        self.create_publisher(Float64, f"{self.topic_prefix}/{metric}/{topic}", 10))
            if has_sign:
                setattr(self, f"_{metric}_absolute_average_pub",
                        self.create_publisher(Float64, f"{self.topic_prefix}/{metric}/absolute_average", 10))

        # Other metrics publisher creation
        self._average_velocity_pub = self.create_publisher(
            Float64, f"{self.topic_prefix}/average_velocity", 10
        )
        self._average_lap_time_pub = self.create_publisher(
            Float64, f"{self.topic_prefix}/average_lap_time", 10
        )
        self._lap_time_pub = self.create_publisher(
            Float64, f"{self.topic_prefix}/lap_time", 10
        )


        # RunningStats initialization
        for metric in self.metric_has_sign.keys():
            setattr(self, f"_{metric}_stats", RunningStats())

        self._average_velocity_stats = RunningStats()
        self._lap_time_stats = RunningStats()

        # Data initialization
        self._has_pose = False
        self._has_speed = False
        self._has_received_control_command = False
        self._has_received_path = False

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_heading = 0.0
        self._current_speed = 0.0
        self._path_points = []
        self._last_lap_stamp = None
        self._last_left_throttle: Optional[float] = None
        self._last_right_throttle: Optional[float] = None
        self._last_steering: Optional[float] = None
        self._current_left_throttle: Optional[float] = None
        self._current_right_throttle: Optional[float] = None
        self._current_steering: Optional[float] = None
        self._left_throttle_updated = False
        self._right_throttle_updated = False
        self._steering_updated = False
        self._cached_path_ref = None

        # Subscribers
        self._path_sub = self.create_subscription(
            PathPointArray,
            "/path_planning/path",
            self._path_callback,
            10,
        )

        self._lap_counter_sub = self.create_subscription(
            Float64,
            "/state_estimation/lap_counter",
            self._lap_counter_callback,
            10,
        )

        self.create_timer(0.05, self._compute_and_publish_metrics)
        self.get_logger().info("control_evaluator node started")


    # Callbacks
    def _path_callback(self, msg: PathPointArray) -> None:
        self._path_points = [
            (point.x, point.y, point.v) for point in msg.pathpoint_array
        ]
        self._has_received_path = True

    # Metrics
    def _update_throttle_smoothness(self, left_throttle: float, right_throttle: float) -> None:
        if self._last_left_throttle is None:
            self._last_left_throttle = left_throttle
        else:
            self._left_throttle_updated = True
        self._current_left_throttle = left_throttle

        if self._last_right_throttle is None:
            self._last_right_throttle = right_throttle
        else:
            self._right_throttle_updated = True
        self._current_right_throttle = right_throttle

    def _update_steering_smoothness(self, steering: float) -> None:
        if self._last_steering is None:
            self._last_steering = steering
        else:
            self._steering_updated = True
        self._current_steering = steering

    def _lap_counter_callback(self, msg: Float64) -> None:
        if not self._has_received_control_command:
            return

        now = self.get_clock().now()

        if self._last_lap_stamp is None:
            self._last_lap_stamp = now
            return

        if self._last_lap_stamp is not None:
            lap_time = (now - self._last_lap_stamp).nanoseconds * 1e-9
            if lap_time > 0.0:
                self._lap_time_stats.add(lap_time)
            self._last_lap_stamp = now

    def _compute_and_publish_metrics(self) -> None:
        self._cached_path_ref = None
        for metric, (_, can_compute, compute_fn) in self.metric_has_sign.items():
            if not can_compute():
                continue
            value = compute_fn()
            stats = getattr(self, f"_{metric}_stats")
            stats.add(value)
            self._publish_metric_set(metric, value, stats)

    def _get_path_reference(self):
        if self._cached_path_ref is None:
            self._cached_path_ref = self._interpolated_path_reference()
        return self._cached_path_ref

    def _compute_left_throttle_smoothness(self) -> float:
        delta = self._current_left_throttle - self._last_left_throttle
        self._last_left_throttle = self._current_left_throttle
        self._left_throttle_updated = False
        return delta

    def _compute_right_throttle_smoothness(self) -> float:
        delta = self._current_right_throttle - self._last_right_throttle
        self._last_right_throttle = self._current_right_throttle
        self._right_throttle_updated = False
        return delta

    def _compute_steering_smoothness(self) -> float:
        delta = self._current_steering - self._last_steering
        self._last_steering = self._current_steering
        self._steering_updated = False
        return delta

    def _find_closest_path_point(self):
        best_index = 0
        best_dist_sq = float("inf")

        for index, (path_x, path_y, path_v) in enumerate(self._path_points):
            dx = self._current_x - path_x
            dy = self._current_y - path_y
            dist_sq = dx * dx + dy * dy
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_index = index

        return best_index

    def _interpolated_path_reference(self):
        if len(self._path_points) == 1:
            point_x, point_y, point_v = self._path_points[0]
            distance = math.sqrt(
                (self._current_x - point_x) * (self._current_x - point_x)
                + (self._current_y - point_y) * (self._current_y - point_y)
            )
            return distance, 0.0, point_v

        closest_index = self._find_closest_path_point()
        candidates = []

        if closest_index > 0:
            start = self._path_points[closest_index - 1]
            end = self._path_points[closest_index]
            candidates.append((start, end))

        if closest_index < len(self._path_points) - 1:
            start = self._path_points[closest_index]
            end = self._path_points[closest_index + 1]
            candidates.append((start, end))

        best = None
        for start, end in candidates:
            distance, interpolation = self._distance_and_interpolation_to_segment(start, end)
            if best is None or distance < best[0]:
                best = (distance, interpolation, start, end)

        if best is None:
            point_x, point_y, point_v = self._path_points[closest_index]
            distance = math.sqrt(
                (self._current_x - point_x) * (self._current_x - point_x)
                + (self._current_y - point_y) * (self._current_y - point_y)
            )
            return distance, 0.0, point_v

        distance, interpolation, start, end = best
        start_x, start_y, start_v = start
        end_x, end_y, end_v = end

        ref_heading = math.atan2(end_y - start_y, end_x - start_x)
        ref_velocity = start_v + interpolation * (end_v - start_v)
        return distance, ref_heading, ref_velocity

    def _distance_and_interpolation_to_segment(self, start, end):
        start_x, start_y, _ = start
        end_x, end_y, _ = end

        segment_x = end_x - start_x
        segment_y = end_y - start_y
        segment_len_sq = segment_x * segment_x + segment_y * segment_y

        if segment_len_sq <= 1e-12:
            distance = math.sqrt(
                (self._current_x - start_x) * (self._current_x - start_x)
                + (self._current_y - start_y) * (self._current_y - start_y)
            )
            return distance, 0.0

        interpolation = (
            (self._current_x - start_x) * segment_x
            + (self._current_y - start_y) * segment_y
        ) / segment_len_sq
        interpolation = max(0.0, min(1.0, interpolation))

        proj_x = start_x + interpolation * segment_x
        proj_y = start_y + interpolation * segment_y
        distance = math.sqrt(
            (self._current_x - proj_x) * (self._current_x - proj_x)
            + (self._current_y - proj_y) * (self._current_y - proj_y)
        )

        return distance, interpolation

    def _reference_heading_at(self, index: int) -> float:
        if len(self._path_points) < 2:
            return 0.0

        if index <= 0:
            x1, y1, _ = self._path_points[0]
            x2, y2, _ = self._path_points[1]
        elif index >= len(self._path_points) - 1:
            x1, y1, _ = self._path_points[-2]
            x2, y2, _ = self._path_points[-1]
        else:
            x1, y1, _ = self._path_points[index - 1]
            x2, y2, _ = self._path_points[index + 1]

        return math.atan2(y2 - y1, x2 - x1)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _publish_metric_set(self, metric: str, instant_value: float, stats: RunningStats) -> None:
        has_sign = self.metric_has_sign[metric][0]
        self._publish_float(getattr(self, f"_{metric}_instant_pub"), instant_value)
        if has_sign:
            self._publish_float(getattr(self, f"_{metric}_average_pub"), stats.average())
            self._publish_float(getattr(self, f"_{metric}_absolute_average_pub"), stats.average_abs())
        else:
            self._publish_float(getattr(self, f"_{metric}_average_pub"), stats.average_abs())
        self._publish_float(getattr(self, f"_{metric}_mse_pub"), stats.mse())

    @staticmethod
    def _publish_float(publisher, value: float) -> None:
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)


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
