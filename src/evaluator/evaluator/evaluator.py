import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ConeArray
from evaluator.fsds_adapter import FSDSAdapter
import message_filters
import numpy as np
from std_msgs.msg import Float32

class Evaluator(Node):

    def __init__(self):
        super().__init__('evaluator')
        self.get_logger().info("Evaluator Node has started")

        self.perception_subscription = message_filters.Subscriber(
            self,
            ConeArray,
            'cones',
        )

        self.perception_mean_difference = self.create_publisher(Float32, '/perception/metrics/mean_difference', 10)
        self.perception_inter_cones_distance = self.create_publisher(Float32, '/perception/metrics/inter_cones_distance', 10)

        self.adater = FSDSAdapter(self, '/lidar/Lidar1')

    
    def compute_and_publish_perception(self):

        mean_difference = Float32()
        mean_difference.data = self.get_average_difference(self.perception_output, self.perception_ground_truth)

        inter_cones_distance = Float32()
        inter_cones_distance.data = self.get_inter_cones_distance(self.perception_output)

        self.perception_mean_difference.publish(mean_difference)

        self.perception_inter_cones_distance.publish(inter_cones_distance)


    @staticmethod
    def get_average_difference(perception_output, perception_ground_truth):
        sum = 0
        count = 0
        for perception_cone in perception_output:
            
            minDistance = np.linalg.norm(perception_cone - perception_ground_truth[0])

            for ground_truth_cone in perception_ground_truth:
                distance = np.linalg.norm(perception_cone - ground_truth_cone)

                if distance < minDistance:
                    minDistance = distance

            sum += minDistance
            count += 1
        
        average = sum / count

        return average
    
    @staticmethod
    def get_inter_cones_distance(perception_output):
        size = len(perception_output)
        visited = set()
        total_distance = 0
        num_pairs = 0

        for i in range(size):
            if i not in visited:
                nearest_distance = float('inf')
                nearest_index = None
                for j in range(size):
                    if j != i and j not in visited:
                        distance = np.linalg.norm(perception_output[i] - perception_output[j])
                        if distance < nearest_distance:
                            nearest_distance = distance
                            nearest_index = j
                if nearest_index is not None:
                    visited.add(i)
                    visited.add(nearest_index)
                    total_distance += nearest_distance
                    num_pairs += 1
        
        if num_pairs == 0:
            return 0
        
        average_distance = total_distance / num_pairs
        return average_distance




def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()