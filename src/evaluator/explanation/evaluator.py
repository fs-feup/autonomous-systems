import rclpy
from rclpy.node import Node
from custom_interfaces.msg import (
    ConeArray,
    VehicleState,
    PathPointArray,
    PathPoint,
    VehicleState,
    EvaluatorControlData,
)
from evaluator.adapter import Adapter
from evaluator.metrics import (
    get_mean_squared_difference,
    get_average_difference,
    get_inter_cones_distance,
    compute_distance,
    get_false_positives,
    get_duplicates,
)
from evaluator.formats import (
    format_vehicle_state_msg,
    format_point2d_msg,
)
import tf2_ros
import sys
from tf2_ros.transform_listener import TransformListener
from evaluator.adapter_maps import (
    ADAPTER_CONSTRUCTOR_DICTINARY,
    ADAPTER_POINT_CLOUD_TOPIC_DICTIONARY,
    ADAPTER_POINT_CLOUD_TOPIC_QOS_DICTIONARY,
)
import message_filters
import numpy as np
import rclpy.subscription
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension, Int32, Float64
from sensor_msgs.msg import PointCloud2
import datetime
from math import sqrt
from evaluator.formats import format_path_point_array_msg
from message_filters import TimeSynchronizer
import csv
import signal

class Evaluator(Node):
    """!
    A ROS2 node for computing and publishing the system's real-time metrics
    """

    def __init__(self) -> None:
        """
        Initializes the Evaluator Node.
        This function sets up the necessary parameters, subscriptions, and publishers for the Evaluator Node.
        It also initializes variables for storing metrics over time.
        Parameters:
            None
        Returns:
            None
        """

    def signal_handler(self, sig: int, frame) -> None:
        """!
        Writes metrics to csv and exits when Ctrl+C is pressed.
        This function is triggered when the program receives a termination signal (e.g., SIGINT or SIGTERM)
        and it saves the collected metrics to CSV files, then exits the program with a status code of 0.
        The metrics are saved in separate CSV files based on their category.

        Args:
            sig (int): The signal number. (not used in this function)
            frame (frame): The current stack frame. (not used in this function)
        Note:
            - Metrics are saved in the "performance/evaluator_metrics" directory as "<metric_name>_<timestamp>.csv".
            - If a category has no metrics, no CSV file will be created for that category.
        Example:
            # Create an instance of the Evaluator class
            evaluator = Evaluator()
            # Register the signal handler function
            signal.signal(signal.SIGINT, evaluator.signal_handler)
            # Start the program
            evaluator.run()
        """

    def metrics_to_csv(self, metrics: list, filename: str) -> None:
        """
        Converts metrics to csv and writes them to a file.
        Args:
            metrics (list): A list of dictionaries representing the metrics.
            filename (str): The name of the CSV file to write the metrics to.
        Returns:
            None
        """

    def correction_step_time_callback(self, msg: Float64) -> None:
        """!
        Callback function to store the execution time of the correction step.

        Args:
            msg (Float64): Message containing the correction step execution time.
        Returns:
            None
        """

    def prediction_step_time_callback(self, msg: Float64) -> None:
        """
        Callback function to store the execution time of the prediction step.
        Args:
            msg (Float64): Message containing the prediction step execution time.
        Returns:
            None
        """

    def perception_execution_time_callback(self, msg: Float64) -> None:
        """!
        Callback function to store the perception execution time.

        Args:
            msg (Float64): Message containing the Perception execution time.
        Returns:
            None
        """

    def planning_execution_time_callback(self, msg: Float64) -> None:
        """!
        Callback function to store the planning execution time.

        Args:
            msg (Float64): Message containing the planning execution time.
        Returns:
            None
        """

    def compute_and_publish_state_estimation(
        self,
        pose: np.ndarray,
        groundtruth_pose: np.ndarray,
        velocities: np.ndarray,
        groundtruth_velocities: np.ndarray,
        map: np.ndarray,
        groundtruth_map: np.ndarray,
    ) -> None:
        """!
        Computes state estimation metrics and publishes them.

        Args:
            pose (np.ndarray): Vehicle state estimation data. [x,y,theta]
            groundtruth_pose (np.ndarray): Ground truth vehicle state data. [x,y,theta]
            veloevaluator-eufs.launch.pyndtruth_velocities (np.ndarray): Ground truth vehicle state velocities. [vx, vy, w]
            map (np.ndarray): Map data. [[x,y,color,confidence]]
            groundtruth_map (np.ndarray): Ground truth map data. [[x,y,color,confidence]]
        """

    def compute_and_publish_perception(
        self, perception_output: np.ndarray, perception_ground_truth: np.ndarray
    ) -> None:
        """!
        Computes perception metrics and publishes them.

        Args:
            perception_output (np.ndarray): Perceived cones.
            perception_ground_truth (np.ndarray): Ground truth cones.
        """

    def compute_and_publish_planning(self, msg: PathPointArray):
        """!
        Computes planning metrics and publishes them.

        Args:
            msg (PathPointArray): Path calculated by the Planning module.
        """

    def planning_gt_callback(self, msg: PathPointArray):
        """!
        Stores the path planning ground truth from mocker node.

        Args:
            msg (PathPointArray): message containing path ground truth.
        """

    def compute_and_publish_control(self, msg: EvaluatorControlData):
        """!
        Computes control metrics and publishes them.
        Args:
            msg (EvaluatorControlData): Control info data message.
        """

def main(args=None):
    """
    Main function for the evaluator module.
    Args:
        args (list): Optional command-line arguments.
    """