from sensor_msgs.msg import PointCloud2
import message_filters

class Adapter:
    """
    Class for subscribing to a point cloud topic and synchronizing messages with a ROS2 node.
    """

    def __init__(self, node, point_cloud_topic):
        """
        Initializes the Adapter.

        Args:
            node (Node): ROS2 node instance.
            point_cloud_topic (str): Topic for point cloud data.
        """
        self.node = node
        self.point_cloud_topic = point_cloud_topic

        # Subscription to point cloud messages
        self.node.point_cloud_subscription = message_filters.Subscriber(
            self.node,
            PointCloud2,
            point_cloud_topic
        )
