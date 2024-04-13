import rclpy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.node import Node
from custom_interfaces.msg import PointArray, ConeArray, Pose as PoseMsg
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2
from evaluator.perception_adapter import PerceptionAdapterROSBag, PerceptionAdapterFSDS
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

        self.perception_points = []
        self.perception_ground_truth = []

        self.perception_subscription = self.create_subscription(
            ConeArray,
            'cones',
            self.perception_callback,
            10
        )

        self.adater = PerceptionAdapterFSDS(self, 'hesai/pandar')

    
    def perception_callback(self, msg: ConeArray):
        self.get_logger().info("Perception Received")
        self.perception_points = []
        for cone in msg.cone_array:
            self.perception_points.append(Cone(cone.position.x, cone.position.y, cone.color))

    def perception_evaluation(self, msg1, msg2, msg3):
        self.get_logger().info("Creating the metrics...")
        sum = 0
        for perception_cone in self.perception_points:
            closest = self.perception_ground_truth[0]
            closest_distance = perception_cone.distance_to(closest)
            for ground_truth_cone in self.perception_ground_truth:
                if (perception_cone.distance_to(ground_truth_cone) < closest_distance):
                    closest = ground_truth_cone
                    closest_distance = perception_cone.distance_to(ground_truth_cone)
            sum += closest_distance
        self.get_logger().info(sum)

def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()