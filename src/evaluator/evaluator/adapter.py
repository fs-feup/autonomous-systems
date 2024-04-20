from sensor_msgs.msg import PointCloud2
import message_filters


class Adapter:
    def __init__(self, node, point_cloud_topic):
        self.node = node
        self.point_cloud_topic = point_cloud_topic
        self.node.point_cloud_subscription = message_filters.Subscriber(
            self.node,
            PointCloud2,
            point_cloud_topic
        )