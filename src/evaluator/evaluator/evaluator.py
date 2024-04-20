import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ConeArray
from evaluator.fsds_adapter import FSDSAdapter
import message_filters
import numpy as np
from std_msgs.msg import Float32

class Evaluator(Node):
    """
    A ROS2 node for computing and publishing system's metrics
    """

    def __init__(self):
        """
        Initializes the Evaluator node and creates the subscriptions/adapters
        """
        super().__init__('evaluator')
        self.get_logger().info("Evaluator Node has started")

        # Subscription to cone array messages (from perception)
        self.perception_subscription = message_filters.Subscriber(
            self,
            ConeArray,
            'cones',
        )

        # Publishers for perception metrics
        self.perception_mean_difference = self.create_publisher(Float32, '/perception/metrics/mean_difference', 10)
        self.perception_inter_cones_distance = self.create_publisher(Float32, '/perception/metrics/inter_cones_distance', 10)

        # FSDS adapter for lidar data
        self.adapter = FSDSAdapter(self, '/lidar/Lidar1')

    
    def compute_and_publish_perception(self):
        """
        Computes perception metrics and publishes them.
        """
        mean_difference = Float32()
        mean_difference.data = self.get_average_difference(self.perception_output, self.perception_ground_truth)

        inter_cones_distance = Float32()
        inter_cones_distance.data = self.get_inter_cones_distance(self.perception_output)

        # Publishes computed perception metrics
        self.perception_mean_difference.publish(mean_difference)
        self.perception_inter_cones_distance.publish(inter_cones_distance)


    @staticmethod
    def get_average_difference(perception_output, perception_ground_truth):
        """
        Computes the average difference between perception output and ground truth cones.
        
        Args:
            perception_output (list): List of perceived cones.
            perception_ground_truth (list): List of ground truth cones.

        Returns:
            float: Average difference between perception output and ground truth cones.
        """
        sum = 0
        count = 0

        if (len(perception_output) == 0):
            return float('inf')
        
        if (len(perception_ground_truth) == 0):
            raise ValueError("No ground truth cones provided for computing average difference.")

        for perception_cone in perception_output:
            min_distance = np.linalg.norm(perception_cone - perception_ground_truth[0])

            for ground_truth_cone in perception_ground_truth:
                distance = np.linalg.norm(perception_cone - ground_truth_cone)
                if distance < min_distance:
                    min_distance = distance

            sum += min_distance
            count += 1
        
        average = sum / count
        return average
    
    @staticmethod
    def get_inter_cones_distance(perception_output):
        """
        Computes the average distance between pairs of perceived cones using Minimum Spanning Tree Prim's algorithm.

        Args:
            perception_output (list): List of perceived cones, where each cone is represented as a numpy array.

        Returns:
            float: Average distance between pairs of perceived cones.
        """
        size = len(perception_output)

        visited = set()

        total_distance = 0
        num_pairs = 0

        adjacency_matrix = np.zeros((size, size))

        for i in range(size):
            for j in range(i + 1, size):
                distance_ij = np.linalg.norm(perception_output[i] - perception_output[j])
                adjacency_matrix[i][j] = distance_ij
                adjacency_matrix[j][i] = distance_ij

        mst = {0}
        visited.add(0)

        while len(mst) < size:
            min_edge = None
            min_distance = float('inf')

            for node in mst:
                for neighbor in range(size):
                    if neighbor not in visited and adjacency_matrix[node][neighbor] < min_distance:
                        min_distance = adjacency_matrix[node][neighbor]
                        min_edge = (node, neighbor)

            if min_edge is not None:
                mst.add(min_edge[1])
                visited.add(min_edge[1])
                total_distance += min_distance
                num_pairs += 1

        if num_pairs == 0:
            return 0
        else:
            average_distance = total_distance / num_pairs
            return average_distance


def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
