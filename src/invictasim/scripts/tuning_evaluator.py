#!/usr/bin/env python3

import csv
import math
from pathlib import Path
import time

import rclpy
from custom_interfaces.msg import ControlCommand, Pose, Velocities
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_pose(pose, origin):
    dx = pose.x - origin.x
    dy = pose.y - origin.y
    cos_yaw = math.cos(origin.theta)
    sin_yaw = math.sin(origin.theta)
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
        normalize_angle(pose.theta - origin.theta),
    )


def transform_pose_to_map(pose, source_origin, target_origin):
    x, y, yaw = relative_pose(pose, source_origin)
    cos_yaw = math.cos(target_origin.theta)
    sin_yaw = math.sin(target_origin.theta)
    return (
        target_origin.x + cos_yaw * x - sin_yaw * y,
        target_origin.y + sin_yaw * x + cos_yaw * y,
        normalize_angle(target_origin.theta + yaw),
    )


class RunningRmse:
    def __init__(self):
        self.sum_squares = 0.0
        self.count = 0

    def update(self, value):
        self.sum_squares += value * value
        self.count += 1
        return math.sqrt(self.sum_squares / self.count)


class TuningEvaluator(Node):
    def __init__(self):
        super().__init__("invictasim_tuning_evaluator")

        self.declare_parameter("output_directory", "src/invictasim/runs")
        self.declare_parameter("path_sample_distance", 0.25)
        self.output_directory = Path(
            self.get_parameter("output_directory").get_parameter_value().string_value
        )
        self.path_sample_distance = (
            self.get_parameter("path_sample_distance").get_parameter_value().double_value
        )

        self.latest_real_pose = None
        self.latest_sim_pose = None
        self.latest_real_velocity = None
        self.latest_sim_velocity = None
        self.real_origin = None
        self.sim_origin = None
        self.start_time = None
        self.csv_file = None
        self.csv_writer = None
        self.last_real_path_point = None
        self.last_sim_path_point = None

        self.position_rmse = RunningRmse()
        self.heading_rmse = RunningRmse()
        self.velocity_rmse = RunningRmse()
        self.yaw_rate_rmse = RunningRmse()

        self.real_path = RosPath()
        self.real_path.header.frame_id = "map"
        self.sim_path = RosPath()
        self.sim_path.header.frame_id = "map"

        self.position_error_pub = self.create_publisher(
            Float64, "/invictasim/tuning/position_error", 10
        )
        self.position_rmse_pub = self.create_publisher(
            Float64, "/invictasim/tuning/position_rmse", 10
        )
        self.heading_error_pub = self.create_publisher(
            Float64, "/invictasim/tuning/heading_error", 10
        )
        self.heading_rmse_pub = self.create_publisher(
            Float64, "/invictasim/tuning/heading_rmse", 10
        )
        self.velocity_error_pub = self.create_publisher(
            Float64, "/invictasim/tuning/velocity_error", 10
        )
        self.velocity_rmse_pub = self.create_publisher(
            Float64, "/invictasim/tuning/velocity_rmse", 10
        )
        self.yaw_rate_error_pub = self.create_publisher(
            Float64, "/invictasim/tuning/yaw_rate_error", 10
        )
        self.yaw_rate_rmse_pub = self.create_publisher(
            Float64, "/invictasim/tuning/yaw_rate_rmse", 10
        )
        self.real_path_pub = self.create_publisher(
            RosPath, "/invictasim/tuning/real_path", 10
        )
        self.sim_path_pub = self.create_publisher(
            RosPath, "/invictasim/tuning/sim_path", 10
        )
        self.real_pose_pub = self.create_publisher(
            PoseStamped, "/invictasim/tuning/real_pose", 10
        )
        self.sim_pose_pub = self.create_publisher(
            PoseStamped, "/invictasim/tuning/sim_pose", 10
        )
        self.visualization_pub = self.create_publisher(
            MarkerArray, "/invictasim/tuning/visualization", 10
        )

        self.create_subscription(
            ControlCommand, "/control/command", self.control_callback, 50
        )
        self.create_subscription(
            Pose, "/state_estimation/vehicle_pose", self.real_pose_callback, 100
        )
        self.create_subscription(
            Pose,
            "/invictasim/state_estimation/vehicle_pose",
            self.sim_pose_callback,
            100,
        )
        self.create_subscription(
            Velocities,
            "/state_estimation/velocities",
            self.real_velocity_callback,
            100,
        )
        self.create_subscription(
            Velocities,
            "/invictasim/state_estimation/velocities",
            self.sim_velocity_callback,
            100,
        )

        self.get_logger().info("Waiting for the first /control/command to align real and sim.")

    def control_callback(self, _msg):
        if self.start_time is not None:
            return
        if self.latest_real_pose is None or self.latest_sim_pose is None:
            self.get_logger().warn(
                "Received control before both pose streams; waiting for the next command."
            )
            return

        self.start_time = time.monotonic()
        self.real_origin = self.latest_real_pose
        self.sim_origin = self.latest_sim_pose
        self.open_csv()
        self.get_logger().info("Tuning evaluation started and trajectories aligned.")

    def real_pose_callback(self, msg):
        self.latest_real_pose = msg
        if self.start_time is not None:
            self.append_path_pose(
                self.real_path,
                transform_pose_to_map(msg, self.real_origin, self.sim_origin),
                real=True,
            )

    def sim_pose_callback(self, msg):
        self.latest_sim_pose = msg
        if self.start_time is None or self.latest_real_pose is None:
            return

        real_x, real_y, real_yaw = transform_pose_to_map(
            self.latest_real_pose, self.real_origin, self.sim_origin
        )
        sim_x, sim_y, sim_yaw = float(msg.x), float(msg.y), float(msg.theta)
        position_error = math.hypot(sim_x - real_x, sim_y - real_y)
        heading_error = normalize_angle(sim_yaw - real_yaw)

        self.position_error_pub.publish(Float64(data=position_error))
        self.position_rmse_pub.publish(
            Float64(data=self.position_rmse.update(position_error))
        )
        self.heading_error_pub.publish(Float64(data=heading_error))
        self.heading_rmse_pub.publish(
            Float64(data=self.heading_rmse.update(heading_error))
        )
        self.append_path_pose(
            self.sim_path, (sim_x, sim_y, sim_yaw), real=False
        )
        self.publish_comparison_visualization(
            real_x, real_y, real_yaw, sim_x, sim_y, position_error
        )
        self.write_csv_row(
            real_x, real_y, real_yaw, sim_x, sim_y, sim_yaw, position_error,
            heading_error
        )

    def real_velocity_callback(self, msg):
        self.latest_real_velocity = msg

    def sim_velocity_callback(self, msg):
        self.latest_sim_velocity = msg
        if self.start_time is None or self.latest_real_velocity is None:
            return

        velocity_error = math.hypot(
            msg.velocity_x - self.latest_real_velocity.velocity_x,
            msg.velocity_y - self.latest_real_velocity.velocity_y,
        )
        yaw_rate_error = msg.angular_velocity - self.latest_real_velocity.angular_velocity
        self.velocity_error_pub.publish(Float64(data=velocity_error))
        self.velocity_rmse_pub.publish(
            Float64(data=self.velocity_rmse.update(velocity_error))
        )
        self.yaw_rate_error_pub.publish(Float64(data=yaw_rate_error))
        self.yaw_rate_rmse_pub.publish(
            Float64(data=self.yaw_rate_rmse.update(yaw_rate_error))
        )

    def append_path_pose(self, path, coordinates, real):
        x, y, yaw = coordinates
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
        pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
        if real:
            self.real_pose_pub.publish(pose_stamped)
        else:
            self.sim_pose_pub.publish(pose_stamped)

        previous = self.last_real_path_point if real else self.last_sim_path_point
        if previous is not None and math.hypot(x - previous[0], y - previous[1]) < (
            self.path_sample_distance
        ):
            return

        path.header.stamp = pose_stamped.header.stamp
        path.poses.append(pose_stamped)
        if real:
            self.last_real_path_point = (x, y)
            self.real_path_pub.publish(path)
        else:
            self.last_sim_path_point = (x, y)
            self.sim_path_pub.publish(path)

    def publish_comparison_visualization(
        self, real_x, real_y, real_yaw, sim_x, sim_y, position_error
    ):
        stamp = self.get_clock().now().to_msg()

        real_car = Marker()
        real_car.header.stamp = stamp
        real_car.header.frame_id = "map"
        real_car.ns = "tuning_real_vehicle"
        real_car.id = 0
        real_car.type = Marker.MESH_RESOURCE
        real_car.action = Marker.ADD
        real_car.pose.position.x = real_x
        real_car.pose.position.y = real_y
        real_car.pose.orientation.z = math.sin(real_yaw / 2.0)
        real_car.pose.orientation.w = math.cos(real_yaw / 2.0)
        real_car.scale.x = 1.0
        real_car.scale.y = 1.0
        real_car.scale.z = 1.0
        real_car.color.r = 0.0
        real_car.color.g = 0.75
        real_car.color.b = 1.0
        real_car.color.a = 0.85
        real_car.mesh_resource = (
            "package://invictasim/resources/meshes/car/02/car_body.glb"
        )
        real_car.mesh_use_embedded_materials = False

        error_line = Marker()
        error_line.header.stamp = stamp
        error_line.header.frame_id = "map"
        error_line.ns = "tuning_position_error"
        error_line.id = 1
        error_line.type = Marker.LINE_LIST
        error_line.action = Marker.ADD
        error_line.pose.orientation.w = 1.0
        error_line.scale.x = 0.06
        error_line.color.r = 1.0
        error_line.color.g = 0.1
        error_line.color.b = 0.1
        error_line.color.a = 1.0
        error_line.points = [
            Point(x=real_x, y=real_y, z=0.4),
            Point(x=sim_x, y=sim_y, z=0.4),
        ]

        distance_text = Marker()
        distance_text.header.stamp = stamp
        distance_text.header.frame_id = "map"
        distance_text.ns = "tuning_position_error"
        distance_text.id = 2
        distance_text.type = Marker.TEXT_VIEW_FACING
        distance_text.action = Marker.ADD
        distance_text.pose.position.x = 0.5 * (real_x + sim_x)
        distance_text.pose.position.y = 0.5 * (real_y + sim_y)
        distance_text.pose.position.z = 1.2
        distance_text.pose.orientation.w = 1.0
        distance_text.scale.z = 0.45
        distance_text.color.r = 1.0
        distance_text.color.g = 1.0
        distance_text.color.b = 1.0
        distance_text.color.a = 1.0
        distance_text.text = f"{position_error:.2f} m"

        self.visualization_pub.publish(
            MarkerArray(markers=[real_car, error_line, distance_text])
        )

    def open_csv(self):
        self.output_directory.mkdir(parents=True, exist_ok=True)
        filename = time.strftime("tuning_%Y%m%d_%H%M%S.csv")
        self.csv_file = (self.output_directory / filename).open(
            "w", newline="", buffering=1
        )
        fieldnames = [
            "elapsed_s", "real_x", "real_y", "real_yaw", "sim_x", "sim_y",
            "sim_yaw", "position_error", "heading_error", "real_velocity_x",
            "real_velocity_y", "real_yaw_rate", "sim_velocity_x",
            "sim_velocity_y", "sim_yaw_rate",
        ]
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.csv_writer.writeheader()
        self.get_logger().info(f"Writing tuning samples to {self.csv_file.name}")

    def write_csv_row(
        self, real_x, real_y, real_yaw, sim_x, sim_y, sim_yaw,
        position_error, heading_error
    ):
        if self.csv_writer is None:
            return
        real_velocity = self.latest_real_velocity
        sim_velocity = self.latest_sim_velocity
        self.csv_writer.writerow(
            {
                "elapsed_s": time.monotonic() - self.start_time,
                "real_x": real_x,
                "real_y": real_y,
                "real_yaw": real_yaw,
                "sim_x": sim_x,
                "sim_y": sim_y,
                "sim_yaw": sim_yaw,
                "position_error": position_error,
                "heading_error": heading_error,
                "real_velocity_x": getattr(real_velocity, "velocity_x", ""),
                "real_velocity_y": getattr(real_velocity, "velocity_y", ""),
                "real_yaw_rate": getattr(real_velocity, "angular_velocity", ""),
                "sim_velocity_x": getattr(sim_velocity, "velocity_x", ""),
                "sim_velocity_y": getattr(sim_velocity, "velocity_y", ""),
                "sim_yaw_rate": getattr(sim_velocity, "angular_velocity", ""),
            }
        )

    def close(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None


def main():
    rclpy.init()
    node = TuningEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
