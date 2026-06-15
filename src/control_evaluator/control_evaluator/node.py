import math
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from custom_interfaces.msg import ControlCommand, PathPointArray, Pose, Velocities
from geometry_msgs.msg import TwistWithCovarianceStamped
from pacsim.msg import StampedScalar, Wheels
from rclpy.node import Node
from std_msgs.msg import Float64


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

    def rmse(self) -> float:
        return math.sqrt(self.mse())


class ControlEvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("control_evaluator")

        self.declare_parameter("global_config_path", "")

        self._adapter = self._load_adapter_from_global_config()
        self.get_logger().info(f"control_evaluator adapter: {self._adapter}")

        self._path_points = []
        self._has_pose = False
        self._has_speed = False
        self._has_received_control_command = False

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_heading = 0.0
        self._current_speed = 0.0

        self._distance_stats = RunningStats()
        self._velocity_error_stats = RunningStats()
        self._heading_error_stats = RunningStats()
        self._average_velocity_stats = RunningStats()
        self._lap_time_stats = RunningStats()

        self._last_lap_counter: Optional[int] = None
        self._last_lap_stamp = None

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

        if self._adapter == "vehicle":
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
        elif self._adapter == "pacsim":
            self._pose_sub = self.create_subscription(
                TwistWithCovarianceStamped,
                "/pacsim/pose",
                self._pacsim_pose_callback,
                10,
            )
            self._vel_sub = self.create_subscription(
                TwistWithCovarianceStamped,
                "/pacsim/velocity",
                self._pacsim_velocity_callback,
                10,
            )
            self._pacsim_steering_control_sub = self.create_subscription(
                StampedScalar,
                "/pacsim/steering_setpoint",
                self._pacsim_control_callback,
                10,
            )
            self._pacsim_throttle_control_sub = self.create_subscription(
                Wheels,
                "/pacsim/throttle_setpoint",
                self._pacsim_control_callback,
                10,
            )
        else:
            self.get_logger().warn(
                f"Unknown adapter '{self._adapter}', defaulting to vehicle topics"
            )
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

        prefix = "/control_evaluator"

        self._distance_instant_pub = self.create_publisher(
            Float64, f"{prefix}/distance_error/instant", 10
        )
        self._distance_average_pub = self.create_publisher(
            Float64, f"{prefix}/distance_error/average", 10
        )
        self._distance_mse_pub = self.create_publisher(
            Float64, f"{prefix}/distance_error/mse", 10
        )
        self._distance_rmse_pub = self.create_publisher(
            Float64, f"{prefix}/distance_error/rmse", 10
        )

        self._velocity_error_instant_pub = self.create_publisher(
            Float64, f"{prefix}/velocity_error/instant", 10
        )
        self._velocity_error_average_pub = self.create_publisher(
            Float64, f"{prefix}/velocity_error/average", 10
        )
        self._velocity_error_signed_average_pub = self.create_publisher(
            Float64, f"{prefix}/velocity_error/signed_average", 10
        )
        self._velocity_error_mse_pub = self.create_publisher(
            Float64, f"{prefix}/velocity_error/mse", 10
        )
        self._velocity_error_rmse_pub = self.create_publisher(
            Float64, f"{prefix}/velocity_error/rmse", 10
        )

        self._heading_error_instant_pub = self.create_publisher(
            Float64, f"{prefix}/heading_error/instant", 10
        )
        self._heading_error_average_pub = self.create_publisher(
            Float64, f"{prefix}/heading_error/average", 10
        )
        self._heading_error_signed_average_pub = self.create_publisher(
            Float64, f"{prefix}/heading_error/signed_average", 10
        )
        self._heading_error_mse_pub = self.create_publisher(
            Float64, f"{prefix}/heading_error/mse", 10
        )
        self._heading_error_rmse_pub = self.create_publisher(
            Float64, f"{prefix}/heading_error/rmse", 10
        )

        self._average_velocity_pub = self.create_publisher(
            Float64, f"{prefix}/average_velocity", 10
        )
        self._average_lap_time_pub = self.create_publisher(
            Float64, f"{prefix}/average_lap_time", 10
        )

        self.get_logger().info("control_evaluator node started")

    def _load_adapter_from_global_config(self) -> str:
        config_path = str(self.get_parameter("global_config_path").value).strip()
        resolved_path = self._resolve_global_config_path(config_path)

        if resolved_path is None:
            self.get_logger().warn(
                "Could not locate config/global/global_config.yaml; using adapter='vehicle'"
            )
            return "vehicle"

        try:
            with resolved_path.open("r", encoding="utf-8") as handle:
                cfg = yaml.safe_load(handle) or {}
            adapter = str(cfg.get("global", {}).get("adapter", "vehicle")).strip()
            return adapter if adapter else "vehicle"
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to read global config at {resolved_path}: {exc}; using adapter='vehicle'"
            )
            return "vehicle"

    def _resolve_global_config_path(self, explicit_path: str) -> Optional[Path]:
        if explicit_path:
            candidate = Path(explicit_path).expanduser()
            if candidate.exists():
                return candidate.resolve()
            self.get_logger().warn(
                f"Configured global_config_path does not exist: {candidate}"
            )

        relative_target = Path("config") / "global" / "global_config.yaml"
        search_roots = [Path.cwd(), Path(__file__).resolve()]
        checked = set()

        for root in search_roots:
            for parent in [root, *root.parents]:
                candidate = (parent / relative_target).resolve()
                if candidate in checked:
                    continue
                checked.add(candidate)
                if candidate.exists():
                    return candidate

        fallback = Path("/home/ws/config/global/global_config.yaml")
        if fallback.exists():
            return fallback

        return None

    def _path_callback(self, msg: PathPointArray) -> None:
        self._path_points = [
            (point.x, point.y, point.v) for point in msg.pathpoint_array
        ]
        self._evaluate_if_ready()

    def _vehicle_pose_callback(self, msg: Pose) -> None:
        self._current_x = msg.x
        self._current_y = msg.y
        self._current_heading = msg.theta
        self._has_pose = True
        self._evaluate_if_ready()

    def _pacsim_pose_callback(self, msg: TwistWithCovarianceStamped) -> None:
        self._current_x = msg.twist.twist.linear.x
        self._current_y = msg.twist.twist.linear.y
        self._current_heading = msg.twist.twist.angular.z
        self._has_pose = True
        self._evaluate_if_ready()

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
        self._evaluate_if_ready()

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
        self._evaluate_if_ready()

    def _vehicle_control_callback(self, _: ControlCommand) -> None:
        self._mark_control_started()

    def _pacsim_control_callback(self, _: object) -> None:
        self._mark_control_started()

    def _mark_control_started(self) -> None:
        if self._has_received_control_command:
            return

        self._has_received_control_command = True
        self.get_logger().info("First control command received; metrics computation started")
        self._evaluate_if_ready()

    def _lap_counter_callback(self, msg: Float64) -> None:
        if not self._has_received_control_command:
            return

        current_lap = int(msg.data)
        now = self.get_clock().now()

        if self._last_lap_counter is None:
            self._last_lap_counter = current_lap
            self._last_lap_stamp = now
            return

        if current_lap > self._last_lap_counter and self._last_lap_stamp is not None:
            lap_time = (now - self._last_lap_stamp).nanoseconds * 1e-9
            if lap_time > 0.0:
                self._lap_time_stats.add(lap_time)
                self._publish_float(self._average_lap_time_pub, self._lap_time_stats.average())
            self._last_lap_stamp = now

        self._last_lap_counter = current_lap

    def _evaluate_if_ready(self) -> None:
        if (
            not self._has_received_control_command
            or not self._path_points
            or not self._has_pose
            or not self._has_speed
        ):
            return

        distance_error, ref_heading, ref_velocity = self._interpolated_path_reference()
        velocity_error = self._current_speed - ref_velocity
        heading_error = self._normalize_angle(self._current_heading - ref_heading)

        self._distance_stats.add(distance_error)
        self._velocity_error_stats.add(velocity_error)
        self._heading_error_stats.add(heading_error)

        self._publish_metric_set(
            distance_error,
            self._distance_stats,
            self._distance_instant_pub,
            self._distance_average_pub,
            self._distance_mse_pub,
            self._distance_rmse_pub,
        )
        self._publish_metric_set(
            velocity_error,
            self._velocity_error_stats,
            self._velocity_error_instant_pub,
            self._velocity_error_average_pub,
            self._velocity_error_mse_pub,
            self._velocity_error_rmse_pub,
            self._velocity_error_signed_average_pub,
        )
        self._publish_metric_set(
            heading_error,
            self._heading_error_stats,
            self._heading_error_instant_pub,
            self._heading_error_average_pub,
            self._heading_error_mse_pub,
            self._heading_error_rmse_pub,
            self._heading_error_signed_average_pub,
        )

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

    def _publish_metric_set(
        self,
        instant_value: float,
        stats: RunningStats,
        instant_pub,
        average_pub,
        mse_pub,
        rmse_pub,
        signed_average_pub=None,
    ) -> None:
        self._publish_float(instant_pub, instant_value)
        self._publish_float(average_pub, stats.average_abs())
        if signed_average_pub is not None:
            self._publish_float(signed_average_pub, stats.average())
        self._publish_float(mse_pub, stats.mse())
        self._publish_float(rmse_pub, stats.rmse())

    @staticmethod
    def _publish_float(publisher, value: float) -> None:
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlEvaluatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
