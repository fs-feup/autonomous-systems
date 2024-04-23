import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ConeArray
from evaluator.fsds_adapter import FSDSAdapter
import message_filters
import numpy as np
from std_msgs.msg import Float32
import datetime
from math import sqrt

class Evaluator(Node):
    """!
    A ROS2 node for computing and publishing system's metrics
    """

    def __init__(self):
        """!
        Initializes the Evaluator node and creates the subscriptions/adapters
        """
        super().__init__('evaluator')
        self.get_logger().info("Evaluator Node has started")

        # Subscription to cone array messages (from perception)
        self.perception_subscription = message_filters.Subscriber(
            self,
            ConeArray,
            'cones'
        )

        self.perception_subscription.registerCallback(self.perception_callback_time_measurement)

        # Publishers for perception metrics
        self.perception_mean_difference = self.create_publisher(Float32, '/perception/metrics/mean_difference', 10)
        self.perception_mean_squared_difference = self.create_publisher(Float32, '/perception/metrics/mean_squared_difference', 10)
        self.perception_root_mean_squared_difference = self.create_publisher(Float32, '/perception/metrics/root_mean_squared_difference', 10)
        self.perception_inter_cones_distance = self.create_publisher(Float32, '/perception/metrics/inter_cones_distance', 10)
        self.perception_execution_time = self.create_publisher(Float32, '/perception/metrics/execution_time', 10)

        # FSDS adapter for lidar data
        self.adapter = FSDSAdapter(self, '/lidar/Lidar1')

    
    # Perception Calback for execution time measurement
    def perception_callback_time_measurement(self, msg : ConeArray):

        """!
        Computes the perception's execution time
        """

        self.get_logger().info("Received perception")
        self.end_time = datetime.datetime.now()
        time_difference = self.end_time - self.start_time
        execution_time = Float32()
        execution_time.data = time_difference
        self.perception_execution_time.publish(execution_time)


    def compute_and_publish_perception(self):
        """!
        Computes perception metrics and publishes them.
        """
        mean_difference = Float32()
        mean_difference.data = self.get_average_difference(self.perception_output, self.perception_ground_truth)

        mean_squared_error = Float32()
        mean_squared_error.data = self.get_mean_squared_error(self.perception_output, self.perception_ground_truth)

        inter_cones_distance = Float32()
        inter_cones_distance.data = self.get_inter_cones_distance(self.perception_output)

        root_mean_squared_difference = Float32()
        root_mean_squared_difference.data = sqrt(self.get_mean_squared_error(self.perception_output)
)

        # Publishes computed perception metrics
        self.perception_mean_difference.publish(mean_difference)
        self.perception_inter_cones_distance.publish(inter_cones_distance)
        self.perception_mean_squared_difference.publish(mean_squared_error)
        self.perception_root_mean_squared_difference.publish(root_mean_squared_difference)


    @staticmethod
    def get_average_difference(perception_output : list, perception_ground_truth : list):
        """!
        Computes the average difference between perception output and ground truth cones.
        
        Args:
            perception_output (list): List of perceived cones.
            perception_ground_truth (list): List of ground truth cones.

        Returns:
            float: Average difference between perception output and ground truth cones.
        """
        sum : float = 0
        count : int = 0

        if (len(perception_output) == 0):
            return float('inf')
        
        if (len(perception_ground_truth) == 0):
            raise ValueError("No ground truth cones provided for computing average difference.")

        for perception_cone in perception_output:
            min_distance : float = np.linalg.norm(perception_cone - perception_ground_truth[0])

            for ground_truth_cone in perception_ground_truth:
                distance : float = np.linalg.norm(perception_cone - ground_truth_cone)
                if distance < min_distance:
                    min_distance = distance

            sum += min_distance
            count += 1
        
        average = sum / count
        return average
    
    @staticmethod
    def get_mean_squared_error(perception_output: list, perception_ground_truth: list):
        """!
        Computes the mean squared error between perception output and ground truth cones.

        Args:
            perception_output (list): List of perceived cones.
            perception_ground_truth (list): List of ground truth cones.

        Returns:
            float: Mean squared error between perception output and ground truth cones.
        """
        if not perception_output:
            raise ValueError("No perception output provided.")
        if not perception_ground_truth:
            raise ValueError("No ground truth cones provided.")

        mse_sum = 0
        for perception_cone in perception_output:
            min_distance_sq = np.linalg.norm(perception_cone - perception_ground_truth[0])**2

            for ground_truth_cone in perception_ground_truth:
                distance_sq = np.linalg.norm(perception_cone - ground_truth_cone)**2
                if distance_sq < min_distance_sq:
                    min_distance_sq = distance_sq

            mse_sum += min_distance_sq

        mse = mse_sum / len(perception_output)
        return mse
    
    @staticmethod
    def get_inter_cones_distance(perception_output : list):
        """!
        Computes the average distance between pairs of perceived cones using Minimum Spanning Tree Prim's algorithm.

        Args:
            perception_output (list): List of perceived cones, where each cone is represented as a numpy array.

        Returns:
            float: Average distance between pairs of perceived cones.
        """
        size : int = len(perception_output)

        visited = set()

        total_distance : int = 0
        num_pairs : int = 0

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
