import rclpy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.node import Node
from custom_interfaces.msg import PointArray, ConeArray, Pose as PoseMsg
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2
from evaluator.perception_adapter import PerceptionAdapterROSBag, PerceptionAdapterFSDS
import message_filters
import math


class Cone:

    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color
    
    def distance_to(self, cone):
        return math.sqrt((self.x-cone.x)**2 + (self.y - cone.y)**2)

class Evaluator(Node):

    def __init__(self):
        super().__init__('evaluator')
        self.get_logger().info("Evaluator Node has started")

        self.perception_subscription = message_filters.Subscriber(
            self,
            ConeArray,
            'cones',
        )

        self.adater = PerceptionAdapterFSDS(self, 'hesai/pandar')


def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()