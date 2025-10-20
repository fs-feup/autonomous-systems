#!/usr/bin/env python3
from __future__ import annotations
import os
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory

from factory_robot.core.factory_map import FactoryMap
from factory_robot.core.factory_robot import Robot


class FactoryNode(Node):
    def __init__(self):
        super().__init__('factory_node')

        # --- params ---
        default_map = os.path.join(
            get_package_share_directory('factory_robot'),
            'maps',
            'factory1.txt'
        )
        self.declare_parameter('map_path', default_map)
        map_path = self.get_parameter('map_path').get_parameter_value().string_value

        self.declare_parameter('publish_rate_hz', 10.0)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.dt = 1.0 / max(1e-6, self.publish_rate_hz)

        # --- state ---
        self.factory_map = FactoryMap(map_path)
        self.robot = Robot(start=self.factory_map.start)

        # --- QoS profiles ---
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- topics ---
        self.map_pub = self.create_publisher(String, 'robot/map', latched_qos)
        self.odom_pub = self.create_publisher(Odometry, 'robot/pose', odom_qos)

        # Visualization
        self.viz_map_pub = self.create_publisher(MarkerArray, 'robot/visualization_map', latched_qos)
        self.viz_pose_pub = self.create_publisher(Marker, 'robot/visualization_pose', latched_qos)

        # Control
        self.cmd_sub = self.create_subscription(String, 'control/cmd', self._on_cmd, cmd_qos)

        # --- timers / cache ---
        self.last_map_str: Optional[str] = None
        self.last_pose: Optional[Tuple[int, int]] = None
        self.timer = self.create_timer(self.dt, self._on_timer)

        # initial publish
        self._publish_all(force=True)
        self.get_logger().info(
            f"Factory node ready. Map size: {self.factory_map.rows}x{self.factory_map.cols}. "
            f"Start at {self.robot.position}. Publishing at {self.publish_rate_hz:.1f} Hz."
        )

    # ---------------- publish loop ----------------
    def _on_timer(self):
        self._publish_all(force=False)
        if self.robot.velocity != (0.0, 0.0):
            self.robot.clear_velocity()

    def _publish_all(self, force: bool):
        # map (without robot)
        map_str = self.factory_map.render_without_robot()
        if force or map_str != self.last_map_str:
            self.map_pub.publish(String(data=map_str))
            self.last_map_str = map_str
            self._publish_map_markers()  # borders, boxes, shelves

        # robot state (odom + robot cube marker)
        r, c = self.robot.position
        pose_changed = (self.last_pose != (r, c))
        if force or pose_changed or self.robot.velocity != (0.0, 0.0):
            self._publish_odom(r, c)
            self._publish_robot_marker(r, c)
            self.last_pose = (r, c)

    # ---------------- odom ----------------
    def _publish_odom(self, r: int, c: int):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose = Pose(
            position=Point(x=float(c), y=float(r), z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        vx, vy = float(self.robot.velocity[1]), float(self.robot.velocity[0])
        msg.twist.twist = Twist(
            linear=Vector3(x=vx, y=vy, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0),
        )
        self.odom_pub.publish(msg)

    # ---------------- visualization: map ----------------
    def _publish_map_markers(self):
        now = self.get_clock().now().to_msg()
        fid = 'map'
        z = 0.05

        arr = MarkerArray()
        marker_id = 0

        # Borders (LINE_STRIP)
        borders = Marker()
        borders.header.frame_id = fid
        borders.header.stamp = now
        borders.ns = 'borders'
        borders.id = marker_id; marker_id += 1
        borders.type = Marker.LINE_STRIP
        borders.action = Marker.ADD
        borders.scale.x = 0.05
        borders.color.r, borders.color.g, borders.color.b, borders.color.a = 0.0, 1.0, 0.0, 1.0
        borders.pose.orientation.w = 1.0
        cols = float(self.factory_map.cols)
        rows = float(self.factory_map.rows)
        borders.points = [
            Point(x=0.0,   y=0.0,   z=z),
            Point(x=cols,  y=0.0,   z=z),
            Point(x=cols,  y=rows,  z=z),
            Point(x=0.0,   y=rows,  z=z),
            Point(x=0.0,   y=0.0,   z=z),
        ]
        arr.markers.append(borders)

        # Shelves (blue cubes)
        for (rr, cc), shelf_ch in self.factory_map.shelf_positions.items():
            m = Marker()
            m.header.frame_id = fid
            m.header.stamp = now
            m.ns = 'shelves'
            m.id = marker_id; marker_id += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.scale.x = 1.0; m.scale.y = 1.0; m.scale.z = 0.12
            m.pose.position.x = float(cc) + 0.5
            m.pose.position.y = float(rr) + 0.5
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.4, 1.0, 0.9
            m.text = shelf_ch
            arr.markers.append(m)

        # Boxes (orange/red cubes)
        for (rr, cc), box_ch in self.factory_map.box_positions.items():
            m = Marker()
            m.header.frame_id = fid
            m.header.stamp = now
            m.ns = 'boxes'
            m.id = marker_id; marker_id += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.scale.x = 1.0; m.scale.y = 1.0; m.scale.z = 0.1
            m.pose.position.x = float(cc) + 0.5
            m.pose.position.y = float(rr) + 0.5
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.2, 0.9
            m.text = box_ch
            arr.markers.append(m)

        self.viz_map_pub.publish(arr)

    # ---------------- visualization: robot as a square ----------------
    def _publish_robot_marker(self, r: int, c: int):
        now = self.get_clock().now().to_msg()
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = now
        m.ns = 'robot'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.scale.x = 1.0; m.scale.y = 1.0; m.scale.z = 0.14
        m.pose.position.x = float(c) + 0.5
        m.pose.position.y = float(r) + 0.5
        m.pose.position.z = 0.07
        m.pose.orientation.w = 1.0
        # distinct color for the robot
        m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.9, 0.1, 1.0
        self.viz_pose_pub.publish(m)

    # ---------------- commands ----------------
    def _on_cmd(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd in ('UP', 'DOWN', 'LEFT', 'RIGHT'):
            self._try_move(cmd)
        elif cmd == 'PICK':
            self._try_pick()
        elif cmd == 'DROP':
            self._try_drop()
        else:
            self.get_logger().warn(f"Unknown command: '{cmd}'")
        self._publish_all(force=False)

    def _try_move(self, direction: str):
        dr, dc = 0, 0
        if direction == 'UP': dr = 1
        elif direction == 'DOWN': dr = -1
        elif direction == 'LEFT': dc = -1
        elif direction == 'RIGHT': dc = +1

        r, c = self.robot.position
        dest = (r + dr, c + dc)
        if not self.factory_map.is_walkable(dest):
            self.get_logger().info(f"Blocked: cannot move {direction} into obstacle at {dest}.")
            return
        self.robot.position = dest
        self.robot.set_velocity_once((dr, dc), step_dt=self.dt)

    def _try_pick(self):
        if self.robot.holding is not None:
            self.get_logger().info("Already holding a box — drop it before picking another.")
            return
        adj = self.factory_map.adjacent_box(self.robot.position)
        if not adj:
            self.get_logger().info("No adjacent box to pick (need 4-neighborhood).")
            return
        (rc, ch) = adj
        removed = self.factory_map.remove_box(rc)
        if removed:
            self.robot.holding = removed
            self.get_logger().info(f"Picked box '{removed}' from {rc}.")
        else:
            self.get_logger().warn("Race? Failed to remove box that was expected to exist.")

    def _try_drop(self):
        if self.robot.holding is None:
            self.get_logger().info("Not holding any box to drop.")
            return
        adj_shelf = self.factory_map.adjacent_shelf(self.robot.position)
        if not adj_shelf:
            self.get_logger().info("Need to be adjacent to a shelf (A/B/C/D) to drop.")
            return
        dropped = self.robot.holding
        self.robot.holding = None
        self.get_logger().info(f"Dropped box '{dropped}' near shelf {adj_shelf[1]} at {adj_shelf[0]}.")


def main(args=None):
    rclpy.init(args=args)
    node = FactoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
