from sensor_msgs.msg import PointCloud2
import message_filters
import datetime
import rclpy


class Adapter:
    """!
    Class for subscribing to a point cloud topic and synchronizing messages with a ROS2 node.
    """

    def __init__(self, node : rclpy.node.Node, point_cloud_topic : str):
        """!
        Initializes the Adapter.

        Args:
            node (Node): ROS2 node instance.
            point_cloud_topic (str): Topic for point cloud data.
        """
        self.node : rclpy.node.Node = node
        self.point_cloud_topic : str = point_cloud_topic

        # Subscription to point cloud messages
        self.node.point_cloud_subscription = message_filters.Subscriber(
            self.node,
            PointCloud2,
            point_cloud_topic
        )

        self.node.point_cloud_subscription.registerCallback(self.point_cloud_callback)

        
    def point_cloud_callback(self, msg : PointCloud2):

        """!
        Point Cloud Callback to get the initiqal time of perception pipeline
        """
        self.node.start_time = datetime.datetime.now()
