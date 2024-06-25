from evaluator.adapter import Adapter
import message_filters
import numpy as np
from sensor_msgs.msg import PointCloud2
from evaluator.formats import format_cone_array_msg
from custom_interfaces.msg import ConeArray
from visualization_msgs.msg import MarkerArray
from evaluator.formats import (
    format_cone_array_msg,
    format_marker_array_msg
)
import rclpy


class RobosenseAdapter(Adapter):

    def __init__(self, node: rclpy.node.Node):

        super().__init__(node)

        self.node.g_truth_subscription_ = message_filters.Subscriber(
            self.node,
            MarkerArray,
            "/perception/visualization/ground_truth/cones",
        )

        self._g_truth = np.array([])

        self._perception_sync_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.perception_subscription_,
                self.node.point_cloud_subscription_,
            ],
            10,
            0.1
        )

        self.node.g_truth_subscription_.registerCallback(self.g_truth)
        self._perception_sync_.registerCallback(self.perception_callback)

    def perception_callback(
        self, perception: ConeArray, point_cloud: PointCloud2
    ):
        if self._g_truth.size != 0:
            perception_output: np.ndarray = format_cone_array_msg(perception)
            self.node.compute_and_publish_perception(
                perception_output, self._g_truth
            )

    def g_truth(self, g_truth : MarkerArray):
        if self._g_truth.size == 0:
            self._g_truth = format_marker_array_msg(g_truth)
    
    