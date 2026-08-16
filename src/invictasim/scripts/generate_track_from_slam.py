#!/usr/bin/env python3

import argparse
from pathlib import Path
import re

import rclpy
import rosbag2_py
import yaml
from custom_interfaces.msg import ConeArray, Pose
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray


DEFAULT_TRACK_NAME = "generated_slam_track"

TRACKS_DIR = Path(__file__).resolve().parents[1] / "resources" / "tracks"

MAP_TOPICS = (
    "/state_estimation/map",
    "/invictasim/state_estimation/map",
)
POSE_TOPICS = (
    "/state_estimation/vehicle_pose",
    "/invictasim/state_estimation/vehicle_pose",
)
LAP_COUNTER_TOPICS = (
    "/state_estimation/lap_counter",
    "/invictasim/state_estimation/lap_counter",
)
PLANNING_COLOR_TOPICS = {
    "/path_planning/yellow_cones": "yellow_cone",
    "/path_planning/blue_cones": "blue_cone",
}

COLOR_ALIASES = {
    "blue": "blue_cone",
    "blue_cone": "blue_cone",
    "yellow": "yellow_cone",
    "yellow_cone": "yellow_cone",
    "orange": "orange_cone",
    "orange_cone": "orange_cone",
    "large_orange": "large_orange_cone",
    "large_orange_cone": "large_orange_cone",
    "unknown": "unknown_cone",
    "unknown_cone": "unknown_cone",
}

MATCH_DISTANCE_METERS = 1.0
POSE_SAMPLE_DISTANCE_METERS = 0.2
MAX_TRAJECTORY_MATCH_DISTANCE_METERS = 8.0
MIN_LATERAL_DISTANCE_METERS = 0.2
TIMING_LINE_HALF_WIDTH_METERS = 5.0
ALLOW_UNKNOWN_CONES = False
DROP_UNCLASSIFIED_CONES = True


class SlamTrackGenerator(Node):
    def __init__(
        self,
        track_name=DEFAULT_TRACK_NAME,
        shutdown_after_write=True,
        defer_write=False,
    ):
        super().__init__("slam_track_generator")

        if not re.fullmatch(r"[A-Za-z0-9_-]+", track_name):
            raise ValueError(
                "Track name may only contain letters, numbers, underscores, and hyphens."
            )

        self.track_name = track_name
        self.shutdown_after_write = shutdown_after_write
        self.defer_write = defer_write
        self.start_pose = None
        self.pose_history = []
        self.latest_map = None
        self.lap_counter = 0
        self.has_lap_one = False
        self.has_written = False
        self.colored_cones = {}
        self.topic_subscriptions = []

        for topic in MAP_TOPICS:
            self.topic_subscriptions.append(
                self.create_subscription(ConeArray, topic, self.map_callback, 10)
            )

        for topic in POSE_TOPICS:
            self.topic_subscriptions.append(
                self.create_subscription(Pose, topic, self.pose_callback, 10)
            )

        for topic in LAP_COUNTER_TOPICS:
            self.topic_subscriptions.append(
                self.create_subscription(Float64, topic, self.lap_counter_callback, 10)
            )

        for topic, color in PLANNING_COLOR_TOPICS.items():
            self.topic_subscriptions.append(
                self.create_subscription(
                    MarkerArray,
                    topic,
                    lambda msg, cone_color=color: self.planning_color_callback(msg, cone_color),
                    10,
                )
            )

        self.status_timer = self.create_timer(5.0, self.log_waiting_status)
        self.get_logger().info(
            "Listening for SLAM map, vehicle pose, lap counter, and planning cone colors."
        )

    def pose_callback(self, msg):
        if self.start_pose is None:
            self.start_pose = msg
            self.get_logger().info(
                f"Captured start position: x={msg.x:.3f}, y={msg.y:.3f}, theta={msg.theta:.3f}"
            )

        if not self.pose_history:
            self.pose_history.append((float(msg.x), float(msg.y), float(msg.theta)))
        else:
            previous_x, previous_y, _ = self.pose_history[-1]
            distance_sq = (msg.x - previous_x) ** 2 + (msg.y - previous_y) ** 2
            if distance_sq >= POSE_SAMPLE_DISTANCE_METERS**2:
                self.pose_history.append((float(msg.x), float(msg.y), float(msg.theta)))

        self.try_write_track()

    def map_callback(self, msg):
        if len(msg.cone_array) == 0:
            return

        self.latest_map = msg
        self.try_write_track()

    def lap_counter_callback(self, msg):
        self.lap_counter = int(msg.data)
        if self.lap_counter >= 1 and not self.has_lap_one:
            self.has_lap_one = True
            self.pose_history.clear()
            self.get_logger().info("Lap counter reached 1. Track generation is now armed.")
        self.try_write_track()

    def planning_color_callback(self, msg, color):
        if len(msg.markers) == 0:
            return

        self.colored_cones[color] = [
            (marker.pose.position.x, marker.pose.position.y) for marker in msg.markers
        ]
        self.try_write_track()

    def try_write_track(self):
        if self.defer_write or self.has_written or not self.has_lap_one:
            return

        if self.start_pose is None or self.latest_map is None:
            return

        cones, unknown_count = self.build_cones()
        if unknown_count > 0 and not ALLOW_UNKNOWN_CONES:
            self.get_logger().warn(
                f"Waiting for planning colors: {unknown_count} map cones are still unknown."
            )
            return

        if not cones:
            return

        self.write_track_file(cones)
        self.has_written = True
        self.get_logger().info("Track file generated. Shutting down.")
        if self.shutdown_after_write:
            rclpy.shutdown()

    def build_cones(self):
        cones = []
        seen = set()
        unknown_count = 0
        inferred_count = 0
        unknown_details = []

        for cone_msg in self.latest_map.cone_array:
            x = float(cone_msg.position.x)
            y = float(cone_msg.position.y)
            key = (round(x, 3), round(y, 3))
            if key in seen:
                continue

            seen.add(key)
            color = self.find_planning_color(x, y)
            if color is None:
                color = COLOR_ALIASES.get(cone_msg.color, cone_msg.color or "unknown_cone")
            if color == "unknown_cone":
                color = self.infer_color_from_trajectory(x, y)
                if color != "unknown_cone":
                    inferred_count += 1
            if color == "unknown_cone":
                unknown_details.append(
                    f"({x:.3f}, {y:.3f}, is_large={cone_msg.is_large})"
                )
                if DROP_UNCLASSIFIED_CONES:
                    continue
                unknown_count += 1
            cones.append([x, y, color])

        if inferred_count:
            self.get_logger().info(
                f"Inferred {inferred_count} cone colors from the driven trajectory."
            )
        if unknown_details:
            action = "Dropped" if DROP_UNCLASSIFIED_CONES else "Unclassified"
            self.get_logger().warn(
                f"{action} cones without a reliable boundary side: "
                + ", ".join(unknown_details)
            )
        ordered_cones = []
        for color in ("yellow_cone", "blue_cone"):
            boundary = [cone for cone in cones if cone[2] == color]
            ordered_cones.extend(self.order_closed_boundary(boundary))
        ordered_cones.extend(
            cone
            for cone in cones
            if cone[2] not in {"yellow_cone", "blue_cone"}
        )
        return ordered_cones, unknown_count

    def find_planning_color(self, x, y):
        best_color = None
        best_distance_sq = MATCH_DISTANCE_METERS * MATCH_DISTANCE_METERS

        for color, markers in self.colored_cones.items():
            for marker_x, marker_y in markers:
                distance_sq = (x - marker_x) ** 2 + (y - marker_y) ** 2
                if distance_sq <= best_distance_sq:
                    best_distance_sq = distance_sq
                    best_color = color

        return best_color

    def infer_color_from_trajectory(self, x, y):
        import math

        if not self.pose_history:
            return "unknown_cone"

        nearest_pose = min(
            self.pose_history,
            key=lambda pose: (x - pose[0]) ** 2 + (y - pose[1]) ** 2,
        )
        pose_x, pose_y, theta = nearest_pose
        dx = x - pose_x
        dy = y - pose_y
        if dx * dx + dy * dy > MAX_TRAJECTORY_MATCH_DISTANCE_METERS**2:
            return "unknown_cone"

        lateral_distance = -math.sin(theta) * dx + math.cos(theta) * dy
        if abs(lateral_distance) < MIN_LATERAL_DISTANCE_METERS:
            return "unknown_cone"
        return "blue_cone" if lateral_distance > 0.0 else "yellow_cone"

    def order_closed_boundary(self, cones):
        if len(cones) < 3:
            return cones

        def distance(a, b):
            return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

        start_x = float(self.start_pose.x)
        start_y = float(self.start_pose.y)
        start_index = min(
            range(len(cones)),
            key=lambda index: (
                (cones[index][0] - start_x) ** 2
                + (cones[index][1] - start_y) ** 2
            ),
        )
        remaining = list(cones)
        route = [remaining.pop(start_index)]
        while remaining:
            next_index = min(
                range(len(remaining)),
                key=lambda index: distance(route[-1], remaining[index]),
            )
            route.append(remaining.pop(next_index))

        improved = True
        while improved:
            improved = False
            for start in range(1, len(route) - 1):
                before = route[start - 1]
                first = route[start]
                for end in range(start + 1, len(route)):
                    last = route[end]
                    after = route[(end + 1) % len(route)]
                    current_length = distance(before, first) + distance(last, after)
                    reversed_length = distance(before, last) + distance(first, after)
                    if reversed_length + 1e-9 < current_length:
                        route[start : end + 1] = reversed(route[start : end + 1])
                        improved = True
        return route

    def write_track_file(self, cones):
        output_path = TRACKS_DIR / f"{self.track_name}.yaml"
        start_position = [float(self.start_pose.x), float(self.start_pose.y)]
        timing_line = self.timing_line_from_pose(self.start_pose)

        data = {
            "track": {
                "start_position": start_position,
                "timing_line": timing_line,
                "cones": cones,
            }
        }

        TRACKS_DIR.mkdir(parents=True, exist_ok=True)
        with output_path.open("w") as track_file:
            yaml.safe_dump(data, track_file, sort_keys=False, default_flow_style=None)

        self.get_logger().info(f"Wrote {len(cones)} cones to {output_path}")

    @staticmethod
    def timing_line_from_pose(pose):
        import math

        normal_theta = pose.theta + math.pi / 2.0
        dx = TIMING_LINE_HALF_WIDTH_METERS * math.cos(normal_theta)
        dy = TIMING_LINE_HALF_WIDTH_METERS * math.sin(normal_theta)

        return [
            [float(pose.x - dx), float(pose.y - dy)],
            [float(pose.x + dx), float(pose.y + dy)],
        ]

    def log_waiting_status(self):
        if self.has_written:
            return

        self.get_logger().info(
            "Waiting: "
            f"lap_counter={self.lap_counter}, "
            f"start_pose={'yes' if self.start_pose else 'no'}, "
            f"map_cones={len(self.latest_map.cone_array) if self.latest_map else 0}, "
            f"colored_cones={sum(len(cones) for cones in self.colored_cones.values())}"
        )


def generate_from_bag(node, bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    handlers = {}
    handlers.update({topic: (ConeArray, node.map_callback) for topic in MAP_TOPICS})
    handlers.update({topic: (Pose, node.pose_callback) for topic in POSE_TOPICS})
    handlers.update(
        {topic: (Float64, node.lap_counter_callback) for topic in LAP_COUNTER_TOPICS}
    )
    handlers.update(
        {
            topic: (
                MarkerArray,
                lambda msg, cone_color=color: node.planning_color_callback(
                    msg, cone_color
                ),
            )
            for topic, color in PLANNING_COLOR_TOPICS.items()
        }
    )

    available_topics = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    selected_handlers = {
        topic: handler for topic, handler in handlers.items() if topic in available_topics
    }
    if not any(topic in selected_handlers for topic in MAP_TOPICS):
        raise ValueError(f"Bag does not contain a supported SLAM map topic: {bag_path}")
    if not any(topic in selected_handlers for topic in POSE_TOPICS):
        raise ValueError(f"Bag does not contain a supported vehicle pose topic: {bag_path}")
    if not any(topic in selected_handlers for topic in LAP_COUNTER_TOPICS):
        raise ValueError(f"Bag does not contain a supported lap counter topic: {bag_path}")

    while reader.has_next():
        topic, serialized_message, _ = reader.read_next()
        handler = selected_handlers.get(topic)
        if handler is None:
            continue
        message_type, callback = handler
        callback(deserialize_message(serialized_message, message_type))

    node.defer_write = False
    node.try_write_track()
    if not node.has_written:
        raise RuntimeError(
            "Track was not generated. The bag must reach lap 1 and provide a map, "
            "a pose, and colors for every cone."
        )


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description="Generate an InvictaSim track YAML from SLAM ROS topics or an MCAP bag."
    )
    parser.add_argument(
        "--track-name",
        default=DEFAULT_TRACK_NAME,
        help="Output track name (written under resources/tracks).",
    )
    parser.add_argument(
        "--bag", type=Path, help="Read topics directly from this MCAP instead of live ROS."
    )
    return parser.parse_args(args)


def main(args=None):
    parsed = parse_args(args)
    rclpy.init()
    node = SlamTrackGenerator(
        track_name=parsed.track_name,
        shutdown_after_write=parsed.bag is None,
        defer_write=parsed.bag is not None,
    )
    try:
        if parsed.bag is None:
            rclpy.spin(node)
        else:
            generate_from_bag(node, parsed.bag)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted before writing a track file.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
