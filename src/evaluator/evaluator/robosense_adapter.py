from evaluator.adapter import Adapter
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
import datetime
from sensor_msgs.msg import PointCloud2
from evaluator.formats import format_cone_array_msg
from custom_interfaces.msg import ConeArray, VehicleState
from visualization_msgs.msg import MarkerArray
from evaluator.formats import (
    format_vehicle_state_msg,
    format_cone_array_msg,
    format_nav_odometry_msg,
    format_marker_array_msg
)
import rclpy


class RobosenseAdapter(Adapter):

    def __init__(self, node: rclpy.node.Node):

        super().__init__(node)

        self.node.g_truth_subscription_ = message_filters.Subscriber(
            self.node,
            ConeArray,
            "/perception/ground_truth",
        )

        self._perception_sync_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.perception_subscription_,
                self.node.point_cloud_subscription_,
                self.node.g_truth_subscription_,
            ],
            10,
            0.1
        )

        self.node.g_truth_subscription_.registerCallback(self.tentative)

        self.node.point_cloud_subscription_.registerCallback(self.pc)

        self.node.perception_subscription_.registerCallback(self.perception)

        self._perception_sync_.registerCallback(self.perception_callback)

    def perception_callback(
        self, perception: ConeArray, point_cloud: PointCloud2, g_truth: ConeArray
    ):
        self.node.get_logger().info("Syncing...")
        perception_ground_truth : np.ndarray = format_cone_array_msg(g_truth)
        perception_output: np.ndarray = format_cone_array_msg(perception)
        self.node.compute_and_publish_perception(
            perception_output, perception_ground_truth
        )

    def tentative(self, perception):
        self.node.get_logger().info("Received Ground Truth!")

    def pc(self, pc):
        self.node.get_logger().info("Received PC!")

    def perception(self, per):
        self.node.get_logger().info("Received perception")
    
    