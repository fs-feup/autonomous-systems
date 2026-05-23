#!/usr/bin/env python3

from pathlib import Path

import rclpy
import yaml
from custom_interfaces.msg import ConeArray, Pose
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray


# Change only this value before running the script.
TRACK_NAME = "generated_slam_track"

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

MIN_COLORED_CONES = 2
MATCH_DISTANCE_METERS = 1.0
START_LINE_HALF_WIDTH_METERS = 5.0
ALLOW_UNKNOWN_CONES = False


class SlamTrackGenerator(Node):
    def __init__(self):
        super().__init__("slam_track_generator")

        self.start_pose = None
        self.latest_map = None
        self.lap_counter = 0
        self.has_lap_one = False
        self.has_written = False
        self.colored_cones = {}
        self.subscriptions = []

        for topic in MAP_TOPICS:
            self.subscriptions.append(
                self.create_subscription(ConeArray, topic, self.map_callback, 10)
            )

        for topic in POSE_TOPICS:
            self.subscriptions.append(
                self.create_subscription(Pose, topic, self.pose_callback, 10)
            )

        for topic in LAP_COUNTER_TOPICS:
            self.subscriptions.append(
                self.create_subscription(Float64, topic, self.lap_counter_callback, 10)
            )

        for topic, color in PLANNING_COLOR_TOPICS.items():
            self.subscriptions.append(
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
        if self.has_written or not self.has_lap_one:
            return

        if self.start_pose is None or self.latest_map is None:
            return

        colored_count = sum(len(cones) for cones in self.colored_cones.values())
        if colored_count < MIN_COLORED_CONES:
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
        rclpy.shutdown()

    def build_cones(self):
        cones = []
        seen = set()
        unknown_count = 0

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
                unknown_count += 1
            cones.append([x, y, color])

        cones.sort(key=lambda cone: (cone[2], cone[0], cone[1]))
        return cones, unknown_count

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

    def write_track_file(self, cones):
        output_path = TRACKS_DIR / f"{TRACK_NAME}.yaml"
        start_position = [float(self.start_pose.x), float(self.start_pose.y)]
        start_line = self.start_line_from_pose(self.start_pose)

        data = {
            "track": {
                "start_position": start_position,
                "start_line": start_line,
                "cones": cones,
            }
        }

        TRACKS_DIR.mkdir(parents=True, exist_ok=True)
        with output_path.open("w") as track_file:
            yaml.safe_dump(data, track_file, sort_keys=False, default_flow_style=None)

        self.get_logger().info(f"Wrote {len(cones)} cones to {output_path}")

    @staticmethod
    def start_line_from_pose(pose):
        import math

        normal_theta = pose.theta + math.pi / 2.0
        dx = START_LINE_HALF_WIDTH_METERS * math.cos(normal_theta)
        dy = START_LINE_HALF_WIDTH_METERS * math.sin(normal_theta)

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


def main():
    rclpy.init()
    node = SlamTrackGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted before writing a track file.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
