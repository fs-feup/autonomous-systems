#!/usr/bin/env python3
from __future__ import annotations
import os
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory

from factory_robot.core.factory_map import FactoryMap
from factory_robot.core.factory_robot import Robot
from custom_interfaces.msg import RobotStatus, Point2d


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
        # Fixed-rate timer; 10 Hz => 100 ms
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
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- topics ---
        self.map_pub = self.create_publisher(String, 'robot/map', latched_qos)
        self.status_pub = self.create_publisher(RobotStatus, 'robot/status', status_qos)

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
        # Always publish at the configured rate (e.g., 10 Hz)
        self._publish_all(force=True)
        # Clear transient velocity after one publish tick
        if self.robot.velocity != (0.0, 0.0):
            self.robot.clear_velocity()

    def _publish_all(self, force: bool):
        # robot state (status + robot cube marker) — always publish at fixed rate
        r, c = self.robot.position
        # map (with robot overlay 'R') — always publish at fixed rate
        map_str = self._render_map_with_robot(r, c)
        self.map_pub.publish(String(data=map_str))
        self.last_map_str = map_str
        self._publish_map_markers()  # borders, boxes, shelves

        self._publish_status(r, c)
        self._publish_robot_marker(r, c)
        self.last_pose = (r, c)

    # ---------------- odom ----------------
    def _publish_status(self, r: int, c: int):
        # Map grid (row, col) to world (x, y) where y is up.
        rows = float(self.factory_map.rows)
        world_x = float(c)
        world_y = rows - 1.0 - float(r)
        vx, vy = float(self.robot.velocity[0]), float(self.robot.velocity[1])
        msg = RobotStatus()
        msg.position = Point2d(x=world_x, y=world_y)
        msg.velocity = Point2d(x=vx, y=vy)
        msg.holding_box = (self.robot.holding is not None)
        self.status_pub.publish(msg)
        # self.get_logger().debug(
        #     f"status: pose=({world_x:.1f},{world_y:.1f}) vel=({vx:.1f},{vy:.1f}) holding={msg.holding_box}"
        # )

    def _render_map_with_robot(self, r: int, c: int) -> str:
        """Return a text map with the robot drawn as 'R' at (r, c)."""
        base = self.factory_map.render_without_robot().split('\n')
        if 0 <= r < len(base) and 0 <= c < len(base[r]):
            row = list(base[r])
            row[c] = 'R'
            base[r] = ''.join(row)
        return '\n'.join(base)

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

        # Shelves (blue cubes) — flip Y so row 0 is at top
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
            m.pose.position.y = float(self.factory_map.rows - 1 - rr) + 0.5
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.4, 1.0, 0.9
            m.text = shelf_ch
            arr.markers.append(m)

        # Boxes (orange/red cubes) — flip Y
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
            m.pose.position.y = float(self.factory_map.rows - 1 - rr) + 0.5
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
        # Flip Y to align with standard world up
        m.pose.position.y = float(self.factory_map.rows - 1 - r) + 0.5
        m.pose.position.z = 0.07
        m.pose.orientation.w = 1.0
        # distinct color for the robot
        m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.9, 0.1, 1.0
        self.viz_pose_pub.publish(m)

    # ---------------- commands ----------------
    def _on_cmd(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd in ('UP', 'DOWN', 'LEFT', 'RIGHT', 'FRONT', 'BACK'):
            self._try_move(cmd)
        elif cmd == 'PICK':
            self._try_pick()
        elif cmd == 'DROP':
            self._try_drop()
        else:
            self.get_logger().warn(f"Unknown command: '{cmd}'")
        # No immediate publish; timer publishes at 10 Hz

    def _try_move(self, direction: str):
        # Viewer-referential commands (UP/DOWN/LEFT/RIGHT), synonyms FRONT/BACK
        # Grid deltas (dr, dc) where rows increase downward
        # Velocity in world axes (x right, y up): vx=dc, vy=-dr
        dr, dc = 0, 0
        if direction in ('UP', 'FRONT'):
            dr = -1
        elif direction in ('DOWN', 'BACK'):
            dr = +1
        elif direction == 'LEFT':
            dc = -1
        elif direction == 'RIGHT':
            dc = +1

        r, c = self.robot.position
        dest = (r + dr, c + dc)
        if not self.factory_map.is_walkable(dest):
            self.get_logger().info(f"Blocked: cannot move {direction} into obstacle at {dest}.")
            return
        self.robot.position = dest
        vx = float(dc)
        vy = float(-dr)
        self.robot.set_velocity_once((vx, vy))

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
