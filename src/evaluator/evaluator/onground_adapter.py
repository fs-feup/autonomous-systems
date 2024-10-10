from evaluator.adapter import Adapter
import message_filters
import numpy as np
from sensor_msgs.msg import PointCloud2
from evaluator.formats import format_cone_array_msg
from custom_interfaces.msg import ConeArray, PathPointArray
from visualization_msgs.msg import MarkerArray
from evaluator.formats import (
    format_cone_array_msg,
    format_marker_array_msg,
    format_path_point_array_msg,
)
import rclpy


class OnGroundAdapter(Adapter):
    """Adapater class to manage the use of OnGround topics."""

    def __init__(self, node: rclpy.node.Node):
        """
        Initializes the OnGroundAdapter class.
        Args:
            node (rclpy.node.Node): The ROS2 node object.
        """

        super().__init__(node)

        self.blue_cones = None
        self.yellow_cones = None

        self.node.blue_cones_subscription_ = self.node.create_subscription(
            MarkerArray,
            "/path_planning/blue_cones",
            self.blue_cones_callback,
            10,
        )

        self.node.yellow_cones_subscription_ = self.node.create_subscription(
            MarkerArray,
            "/path_planning/yellow_cones",
            self.yellow_cones_callback,
            10,
        )

        self.node.planning_subscription_ = self.node.create_subscription(
            PathPointArray,
            "/path_planning/path",
            self.planning_callback,
            10,
        )

    def blue_cones_callback(self, blue_cones_msg: MarkerArray):
        """Callback for blue cones."""
        self.blue_cones = format_marker_array_msg(blue_cones_msg)
        self.node.get_logger().info("Updated blue cones")

    def yellow_cones_callback(self, yellow_cones_msg: MarkerArray):
        """Callback for yellow cones."""
        self.yellow_cones = format_marker_array_msg(yellow_cones_msg)
        self.node.get_logger().info("Updated yellow cones")

    def planning_callback(self, planning_output_msg: PathPointArray):
        """Callback for planning output."""
        # Ensure blue and yellow cones have been received before processing
        if self.blue_cones is None or self.yellow_cones is None:
            self.node.get_logger().warn("Waiting for cone data...")
            return

        # Process the planning output
        planning_output_treated = format_path_point_array_msg(planning_output_msg)

        self.node.get_logger().info("Processing planning output")
        # Publish the planning results using the stored cones
        self.node.compute_and_publish_planning(
            planning_output_treated,
            planning_output_treated,
            self.blue_cones,
            self.yellow_cones,
        )
