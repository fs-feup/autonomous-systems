import rclpy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.node import Node
from custom_interfaces.msg import PointArray, ConeArray, Pose as PoseMsg
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2, Float32
from evaluator.perception_adapter import PerceptionAdapterROSBag, PerceptionAdapterFSDS
import message_filters
import math
import numpy as np

class Evaluator(Node):

    def __init__(self):
        super().__init__('evaluator')
        self.get_logger().info("Evaluator Node has started")

        self.perception_subscription = message_filters.Subscriber(
            self,
            ConeArray,
            'cones',
        )

        self.perception_mean_difference = self.create_publisher(Float32, '/perception/metrics/mean_difference', QoSProfile(depth=10))
        self.perception_inter_cones_distance = self.create_publisher(Float32, '/perception/metrics/inter_cones_distance', QoSProfile(depth=10))

        self.adater = PerceptionAdapterFSDS(self, 'lidar/Lidar1')

    
    def compute_and_publish_perception(self):

    

        mean_difference = Float32()
        mean_difference.data = self.get_average_difference()

        inter_cones_distance = Float32()
        inter_cones_distance.data = self.get_inter_cones_distance()

        self.perception_mean_difference.publish(mean_difference)

        self.perception_inter_cones_distance(inter_cones_distance)

        self.get_logger(f"First metric: {mean_difference.data}")

    def get_average_difference(self):
        for perception_cone in self.perception_output:
            
            minDistance = np.linalg.norm(perception_cone - self.perception_ground_truth[0])
            for ground_truth_cone in self.perception_ground_truth:

                # Calculates the difference between cones
                distance = np.linalg.norm(perception_cone - ground_truth_cone)

                if distance < minDistance:
                    minDistance = distance

            sum += minDistance
            count += 1
        
        average = sum / count

        return average
    
    def get_inter_cones_distance(self):

        size = len(self.perception_output)
        sum = 0
        count = 0

        for i in range (size):
            for j in range(i+1, size):
                sum += np.linalg.norm(self.perception_output[i] - self.perception_output[j])
                count += 1
        
        average = sum / count

        return average




def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()