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

class VehicleAdapter(Adapter):

    def __init__(self, node: rclpy.node.Node):

    def perception_callback(
        self, perception: ConeArray
    ):

    def g_truth_callback(self, g_truth : MarkerArray):