#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from custom_interfaces.msg import PerceptionOutput, Pose


class ConeMarkerPublisher(Node):
    def __init__(self):
        super().__init__("cone_marker_publisher")

        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_theta = 0.0
        self.have_pose = False

        # Store all global cones seen so far
        self.global_cones = []

        self.create_subscription(
            PerceptionOutput, "/perception/cones", self.cones_callback, 10
        )

        self.create_subscription(
            Pose, "/state_estimation/vehicle_pose", self.pose_callback, 10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, "/visualization/cones_markers", 10
        )

    def pose_callback(self, msg: Pose):
        self.vehicle_x = msg.x
        self.vehicle_y = msg.y
        self.vehicle_theta = msg.theta
        self.have_pose = True

    def cones_callback(self, msg: PerceptionOutput):
        if not self.have_pose:
            return

        cos_t = math.cos(self.vehicle_theta)
        sin_t = math.sin(self.vehicle_theta)

        # Transform new cones to global frame and store them
        for cone in msg.cone_array:
            lx = cone.x
            ly = cone.y

            gx = self.vehicle_x + cos_t * lx - sin_t * ly
            gy = self.vehicle_y + sin_t * lx + cos_t * ly

            self.global_cones.append((gx, gy))

        # Build the map markers with ALL cones observed so far
        marker_array = MarkerArray()
        marker_id = 0

        for gx, gy in self.global_cones:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.SPHERE

            marker.pose.position.x = gx
            marker.pose.position.y = gy
            marker.pose.position.z = 0.0

            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25

            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 1.0

            # Make markers persistent
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            marker_array.markers.append(marker)
            marker_id += 1

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ConeMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
