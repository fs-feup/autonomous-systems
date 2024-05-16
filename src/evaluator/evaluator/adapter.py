from sensor_msgs.msg import PointCloud2
import message_filters
import datetime
import rclpy


class Adapter:
    """!
    Class for subscribing to a point cloud topic and synchronizing messages with a ROS2 node.
    """

    def __init__(self, node: rclpy.Node):
        """!
        Initializes the Adapter.

        Args:
            node (Node): ROS2 node instance.
            point_cloud_topic (str): Topic for point cloud data.
        """
        self.node: rclpy.Node = node
