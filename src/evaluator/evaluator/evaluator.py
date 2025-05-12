import rclpy
from rclpy.node import Node
from custom_interfaces.msg import (
    ConeArray,
    VehicleState,
    PathPointArray,
    PathPoint,
    VehicleState,
    Velocities,
    EvaluatorControlData,
    Pose,
)
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped
from evaluator.adapter import Adapter
from evaluator.metrics import (
    get_mean_squared_difference,
    get_average_difference,
    get_inter_cones_distance,
    compute_distance,
    get_false_positives,
    get_duplicates,
    compute_closest_distances,
    get_average_error,
    get_mean_squared_error,
)
from evaluator.formats import (
    format_vehicle_state_msg,
    format_point2d_msg,
    find_closest_elements,
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
from std_msgs.msg import Float32

import csv
import signal
import yaml
import os
from ament_index_python.packages import get_package_prefix


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
        super().__init__("evaluator")
        self.get_logger().info("Evaluator Node has started")

        # Load configuration from YAML file
        self.load_config()

        if (self._adapter_name_ == "fsds") and (self.use_simulated_perception_):
            self.get_logger().error(
                "Simulated perception is not supported for FSDS adapter"
            )
            sys.exit(1)

        # Subscriptions
        self.perception_subscription_ = message_filters.Subscriber(
            self, ConeArray, "/perception/cones"
        )
        self.map_subscription_ = message_filters.Subscriber(
            self, ConeArray, "/state_estimation/map"
        )
        self.velocities_subscription_ = message_filters.Subscriber(
            self, Velocities, "/state_estimation/velocities"
        )
        self.vehicle_pose_subscription_ = message_filters.Subscriber(
            self, Pose, "/state_estimation/vehicle_pose"
        )
        self.planning_subscription_ = message_filters.Subscriber(
            self, PathPointArray, "/path_planning/path"
        )
        self.transform_buffer_ = tf2_ros.Buffer()
        self._transforms_listener_ = TransformListener(self.transform_buffer_, self)
        self.planning_gt_subscription_ = message_filters.Subscriber(
            self, PathPointArray, "/path_planning/mock_path"
        )
        self.control_data_sub_ = self.create_subscription(
            EvaluatorControlData,
            "/control/evaluator_data",
            self.compute_and_publish_control,
            10,
        )
        self._correction_step_time_subscription_ = self.create_subscription(
            Float64,
            "/state_estimation/execution_time/correction_step",
            self.correction_step_time_callback,
            10,
        )
        self._prediction_step_time_subscription_ = self.create_subscription(
            Float64,
            "/state_estimation/execution_time/prediction_step",
            self.prediction_step_time_callback,
            10,
        )
        self._planning_execution_time_subscription_ = self.create_subscription(
            Float64,
            "/path_planning/execution_time",
            self.planning_execution_time_callback,
            10,
        )
        self._perception_execution_time_subscription_ = self.create_subscription(
            Float64,
            "/perception/execution_time",
            self.perception_execution_time_callback,
            10,
        )

        # Publishers for perception metrics
        self._perception_mean_difference_ = self.create_publisher(
            Float32, "/evaluator/perception/mean_difference", 10
        )
        self._perception_mean_squared_difference_ = self.create_publisher(
            Float32, "/evaluator/perception/mean_squared_difference", 10
        )
        self._perception_root_mean_squared_difference_ = self.create_publisher(
            Float32, "/evaluator/perception/root_mean_squared_difference", 10
        )
        self._perception_inter_cones_distance_ = self.create_publisher(
            Float32, "/evaluator/perception/inter_cones_distance", 10
        )
        self._perception_false_positives_ = self.create_publisher(
            Int32, "/evaluator/perception/false_positives", 10
        )
        self._perception_precision_ = self.create_publisher(
            Float32, "/evaluator/perception/precision", 10
        )
        self._perception_recall_ = self.create_publisher(
            Float32, "/evaluator/perception/recall", 10
        )
        self._perception_number_duplicates = self.create_publisher(
            Int32, "/evaluator/perception/number_duplicates", 10
        )

        # Publishers for state estimation metrics
        self._velocities_difference_ = self.create_publisher(
            Float32MultiArray,
            "/evaluator/state_estimation/velocities_difference",
            10,
        )
        self._vehicle_pose_difference_ = self.create_publisher(
            Float32MultiArray,
            "/evaluator/state_estimation/vehicle_pose_difference",
            10,
        )
        self._map_mean_squared_difference_ = self.create_publisher(
            Float32, "/evaluator/state_estimation/map_mean_squared_difference", 10
        )
        self._map_mean_difference_ = self.create_publisher(
            Float32, "/evaluator/state_estimation/map_mean_difference", 10
        )
        self._map_root_mean_squared_difference_ = self.create_publisher(
            Float32, "/evaluator/state_estimation/map_root_mean_squared_difference", 10
        )

        self._se_false_positives_ = self.create_publisher(
            Int32, "/evaluator/state_estimation/false_positives", 10
        )
        self._se_number_duplicates = self.create_publisher(
            Int32, "/evaluator/state_estimation/number_duplicates", 10
        )
        self._se_difference_with_map_ = self.create_publisher(
            Int32, "/evaluator/state_estimation/difference_with_map", 10
        )

        # Publisher for path planning metrics
        self._planning_mean_difference_to_gt = self.create_publisher(
            Float32, "/evaluator/planning/gt_me", 10
        )
        self._planning_mean_squared_difference_to_gt = self.create_publisher(
            Float32, "/evaluator/planning/gt_mse", 10
        )
        self._planning_root_mean_squared_difference_to_gt = self.create_publisher(
            Float32, "/evaluator/planning/gt_rmse", 10
        )

        self._planning_mean_difference_to_left_cones = self.create_publisher(
            Float32, "/evaluator/planning/left_cones_me", 10
        )
        self._planning_mean_squared_difference_to_left_cones = self.create_publisher(
            Float32, "/evaluator/planning/left_cones_mse", 10
        )
        self._planning_root_mean_squared_difference_to_left_cones = (
            self.create_publisher(Float32, "/evaluator/planning/left_cones_rmse", 10)
        )

        self._planning_mean_difference_to_right_cones = self.create_publisher(
            Float32, "/evaluator/planning/right_cones_me", 10
        )
        self._planning_mean_squared_difference_to_right_cones = self.create_publisher(
            Float32, "/evaluator/planning/right_cones_mse", 10
        )
        self._planning_root_mean_squared_difference_to_right_cones = (
            self.create_publisher(Float32, "/evaluator/planning/right_cones_rmse", 10)
        )

        self._planning_mean_difference_to_cones = self.create_publisher(
            Float32, "/evaluator/planning/cones_me", 10
        )
        self._planning_mean_squared_difference_to_cones = self.create_publisher(
            Float32, "/evaluator/planning/cones_mse", 10
        )
        self._planning_root_mean_squared_difference_to_cones = self.create_publisher(
            Float32, "/evaluator/planning/cones_rmse", 10
        )

        # Publisher for control metrics
        self._control_closest_pose_difference_ = self.create_publisher(
            Float32, "/evaluator/control/pose/closest/difference", 10
        )
        self._control_velocity_closest_difference_ = self.create_publisher(
            Float32, "/evaluator/control/velocity/closest/difference", 10
        )
        self._control_velocity_lookahead_difference_ = self.create_publisher(
            Float32, "/evaluator/control/velocity/lookahead/difference", 10
        )

        # Replace spaces with underscores in csv_suffix
        self.csv_suffix = self.csv_suffix.replace(" ", "_")

        # Metrics over time
        self.perception_metrics = []
        self.map_metrics = []
        self.pose_metrics = []
        self.vel_estimation_metrics = []
        self.planning_metrics = []
        self.control_metrics = []
        self._se_correction_execution_time_ = []
        self._se_prediction_execution_time_ = []
        self._control_execution_time_ = []
        self._planning_execution_time_ = []
        self._perception_execution_time_ = []

        self._control_pose_sum_error = 0
        self._control_pose_squared_sum_error = 0
        self._control_velocity_sum_error = 0
        self._control_velocity_squared_sum_error = 0
        self._control_count = 0
        self._control_closest_velocity_sum_error = 0
        self.control_closest_velocity_squared_sum_error = 0

        self._perception_sum_error = 0
        self._perception_squared_sum_error = 0
        self._perception_root_squared_sum_error = 0
        self._perception_precision_sum = 0
        self._perception_recall_sum = 0
        self._perception_count = 0

        self._se_map_sum_error = 0
        self._se_map_squared_sum_error = 0
        self._se_map_mean_root_squared_sum_error = 0
        self._map_count_ = 0
        self._pose_count_ = 0
        self._ve_count_ = 0

        self._sum_velocities_error = Float32MultiArray()
        self._sum_squared_velocities_error = Float32MultiArray()
        self._sum_velocities_error.layout.dim = [MultiArrayDimension()]
        self._sum_squared_velocities_error.layout.dim = [MultiArrayDimension()]
        self._sum_velocities_error.layout.dim[0].size = 3
        self._sum_squared_velocities_error.layout.dim[0].size = 3
        self._sum_velocities_error.data = [0.0] * 3
        self._sum_squared_velocities_error.data = [0.0] * 3
        self._sum_vehicle_pose_error = Float32MultiArray()
        self._sum_squared_vehicle_pose_error = Float32MultiArray()
        self._sum_vehicle_pose_error.layout.dim = [MultiArrayDimension()]
        self._sum_squared_vehicle_pose_error.layout.dim = [MultiArrayDimension()]
        self._sum_vehicle_pose_error.layout.dim[0].size = 3
        self._sum_squared_vehicle_pose_error.layout.dim[0].size = 3
        self._sum_vehicle_pose_error.data = [0.0] * 3
        self._sum_squared_vehicle_pose_error.data = [0.0] * 3
        self._planning_sum_error = 0
        self._planning_squared_sum_error = 0
        self._planning_mean_root_squared_sum_error = 0
        self._planning_count = 0

        # Perception Metrics Over time
        self._perception_mean_mean_error = self.create_publisher(
            Float32, "/evaluator/perception/mean_mean_error", 10
        )

        self._perception_mean_mean_squared_error = self.create_publisher(
            Float32, "/evaluator/perception/mean_mean_squared_error", 10
        )

        self._perception_mean_mean_root_squared_error = self.create_publisher(
            Float32, "/evaluator/perception/mean_mean_root_squared_error", 10
        )

        self._perception_mean_precision = self.create_publisher(
            Float32, "/evaluator/perception/mean_precision", 10
        )

        self._perception_mean_recall = self.create_publisher(
            Float32, "/evaluator/perception/mean_recall", 10
        )

        # State estimation metrics over time
        self._map_mean_mean_error = self.create_publisher(
            Float32, "/evaluator/state_estimation/map_mean_mean_difference", 10
        )

        self._map_mean_mean_squared_error = self.create_publisher(
            Float32, "/evaluator/state_estimation/map_mean_mean_squared_difference", 10
        )

        self._map_mean_mean_root_squared_error = self.create_publisher(
            Float32,
            "/evaluator/state_estimation/map_mean_mean_root_squared_difference",
            10,
        )

        self._state_estimation_mean_vehicle_pose_error = self.create_publisher(
            Float32MultiArray,
            "/evaluator/state_estimation/vehicle_pose_mean_difference",
            10,
        )

        self._state_estimation_mean_squared_vehicle_pose_error = self.create_publisher(
            Float32MultiArray,
            "/evaluator/state_estimation/vehicle_pose_mean_squared_difference",
            10,
        )

        self._state_estimation_root_mean_squared_vehicle_pose_error = (
            self.create_publisher(
                Float32MultiArray,
                "/evaluator/state_estimation/vehicle_pose_mean_root_squared_difference",
                10,
            )
        )

        self._state_estimation_mean_velocities_error = self.create_publisher(
            Float32MultiArray,
            "/evaluator/state_estimation/velocities_mean_difference",
            10,
        )

        self._state_estimation_mean_squared_velocities_error = self.create_publisher(
            Float32MultiArray,
            "/evaluator/state_estimation/velocities_mean_squared_difference",
            10,
        )

        self._state_estimation_root_mean_squared_velocities_error = (
            self.create_publisher(
                Float32MultiArray,
                "/evaluator/state_estimation/velocities_mean_root_squared_difference",
                10,
            )
        )

        # Planning metrics over time
        self._planning_mean_mean_error = self.create_publisher(
            Float32, "/evaluator/planning/mean_mean_difference", 10
        )

        self._planning_mean_mean_squared_error = self.create_publisher(
            Float32, "/evaluator/planning/mean_mean_squared_difference", 10
        )

        self._planning_mean_mean_root_squared_error = self.create_publisher(
            Float32, "/evaluator/planning/mean_mean_root_squared_difference", 10
        )

        # Control metrics over time
        self._control_pose_difference_mean_ = self.create_publisher(
            Float32, "/evaluator/control/pose/difference_mean", 10
        )

        self._control_pose_mean_squared_difference_ = self.create_publisher(
            Float32, "/evaluator/control/pose/mean_squared_difference", 10
        )

        self._control_pose_root_mean_squared_difference_ = self.create_publisher(
            Float32, "/evaluator/control/pose/root_mean_squared_difference", 10
        )
        self._control_velocity_to_lookahead_velocity_difference_mean_ = (
            self.create_publisher(
                Float32, "/evaluator/control/velocity/lookahead/difference_mean", 10
            )
        )
        self._control_velocity_to_lookahead_velocity_mean_squared_difference_ = (
            self.create_publisher(
                Float32,
                "/evaluator/control/velocity/lookahead/mean_squared_difference",
                10,
            )
        )
        self._control_velocity_to_lookahead_velocity_root_mean_squared_difference_ = (
            self.create_publisher(
                Float32,
                "/evaluator/control/velocity/lookahead/root_mean_squared_difference",
                10,
            )
        )

        self._control_velocity_to_closest_velocity_difference_mean_ = (
            self.create_publisher(
                Float32, "/evaluator/control/velocity/closest/difference_mean", 10
            )
        )
        self._control_velocity_to_closest_velocity_mean_squared_difference_ = (
            self.create_publisher(
                Float32,
                "/evaluator/control/velocity/closest/mean_squared_difference",
                10,
            )
        )

        self._control_velocity_to_closest_velocity_root_mean_squared_difference_ = (
            self.create_publisher(
                Float32,
                "/evaluator/control/velocity/closest/root_mean_squared_difference",
                10,
            )
        )

        # Adapter selection
        self._adapter_: Adapter = ADAPTER_CONSTRUCTOR_DICTINARY[self._adapter_name_](
            self
        )

        signal.signal(signal.SIGINT, self.signal_handler)

    def get_config_yaml_path(self, package_name, dir, filename):
        package_share_directory = get_package_prefix(package_name)
        config_path = os.path.join(
            package_share_directory, "..", "..", "config", dir, f"{filename}.yaml"
        )
        return config_path

    def load_config(self):
        """Load configuration from YAML file."""
        global_config_path = self.get_config_yaml_path(
            "evaluator", "global", "global_config"
        )
        self.get_logger().debug(f"Loading global config from: {global_config_path}")
        with open(global_config_path, "r") as file:
            global_config = yaml.safe_load(file)

        adapter = global_config["global"]["adapter"]
        self._adapter_name_ = adapter
        self.use_simulated_perception_ = global_config["global"][
            "use_simulated_perception"
        ]
        self.use_simulated_se_ = global_config["global"]["use_simulated_se"]
        self.use_simulated_planning_ = global_config["global"]["use_simulated_planning"]

        specific_config_path = self.get_config_yaml_path(
            "evaluator", "evaluator", adapter
        )
        self.get_logger().debug(f"Loading specific config from: {specific_config_path}")
        with open(specific_config_path, "r") as file:
            specific_config = yaml.safe_load(file)

        self.generate_csv = specific_config["evaluator"]["generate_csv"]
        self.csv_suffix = specific_config["evaluator"]["csv_suffix"].replace(" ", "_")

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
        if self.generate_csv:
            finish_time = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            metrics_dict = {
                "perception": self.perception_metrics,
                "map": self.map_metrics,
                "pose": self.pose_metrics,
                "planning": self.planning_metrics,
                "control": self.control_metrics,
                "se_correction_execution_time": self._se_correction_execution_time_,
                "se_prediction_execution_time": self._se_prediction_execution_time_,
                "control_execution_time": self._control_execution_time_,
                "planning_execution_time": self._planning_execution_time_,
                "perception_execution_time": self._perception_execution_time_,
            }
            for filename, metrics in metrics_dict.items():
                if metrics:
                    datetime_filename = f"{filename}_{self._adapter_name_}_{finish_time}_{self.csv_suffix}.csv"
                    self.metrics_to_csv(
                        metrics, "performance/evaluator_metrics/" + datetime_filename
                    )
        sys.exit(0)

    def metrics_to_csv(self, metrics: list, filename: str) -> None:
        """
        Converts metrics to csv and writes them to a file.
        Args:
            metrics (list): A list of dictionaries representing the metrics.
            filename (str): The name of the CSV file to write the metrics to.
        Returns:
            None
        """

        # Add 'time' key to each metric
        start_time = metrics[0]["timestamp"]
        for metric in metrics:
            elapsed_time = (metric["timestamp"] - start_time).total_seconds()
            metric["time"] = elapsed_time  # Add/Update 'time' key with elapsed time

        # Write metrics to csv
        with open(filename, "w", newline="") as csvfile:
            fieldnames = metrics[0].keys()
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(metrics)

    def correction_step_time_callback(self, msg: Float64) -> None:
        """!
        Callback function to store the execution time of the correction step.

        Args:
            msg (Float64): Message containing the correction step execution time.
        Returns:
            None
        """
        self._se_correction_execution_time_.append(
            {"timestamp": datetime.datetime.now(), "execution_time": msg.data}
        )

    def prediction_step_time_callback(self, msg: Float64) -> None:
        """
        Callback function to store the execution time of the prediction step.
        Args:
            msg (Float64): Message containing the prediction step execution time.
        Returns:
            None
        """
        self._se_prediction_execution_time_.append(
            {"timestamp": datetime.datetime.now(), "execution_time": msg.data}
        )

    def perception_execution_time_callback(self, msg: Float64) -> None:
        """!
        Callback function to store the perception execution time.

        Args:
            msg (Float64): Message containing the Perception execution time.
        Returns:
            None
        """
        self._perception_execution_time_.append(
            {"timestamp": datetime.datetime.now(), "execution_time": msg.data}
        )

    def planning_execution_time_callback(self, msg: Float64) -> None:
        """!
        Callback function to store the planning execution time.

        Args:
            msg (Float64): Message containing the planning execution time.
        Returns:
            None
        """
        self._planning_execution_time_.append(
            {"timestamp": datetime.datetime.now(), "execution_time": msg.data}
        )

    def compute_and_publish_pose(
        self,
        pose: np.ndarray,
        groundtruth_pose: np.ndarray,
    ) -> None:
        """!
        Computes state estimation metrics and publishes them.
        Args:
            pose (np.ndarray): Vehicle state estimation data. [x,y,theta]
            groundtruth_pose (np.ndarray): Ground truth vehicle state data. [x,y,theta]
        """

        self.get_logger().debug("Received pose")

        # Compute instantaneous vehicle pose metrics
        vehicle_pose_error = Float32MultiArray()
        vehicle_pose_error.layout.dim = [MultiArrayDimension()]
        vehicle_pose_error.layout.dim[0].size = 3
        vehicle_pose_error.layout.dim[0].label = "Vehicle Pose Error: [x, y, theta]"
        vehicle_pose_error.data = [0.0] * 3
        if groundtruth_pose != []:
            vehicle_pose_error.data[0] = abs(pose[0] - groundtruth_pose[0])
            vehicle_pose_error.data[1] = abs(pose[1] - groundtruth_pose[1])
            vehicle_pose_error.data[2] = abs(pose[2] - groundtruth_pose[2]) % (
                2 * np.pi
            )

        self.get_logger().debug(
            "Computed slam metrics:\n \
                            Vehicle state error: {}".format(
                vehicle_pose_error,
            )
        )

        # Publishes computed state estimation metrics
        self._vehicle_pose_difference_.publish(vehicle_pose_error)

        # Compute vehicle pose metrics over time
        mean_vehicle_pose_error = Float32MultiArray()
        mean_squared_vehicle_pose_error = Float32MultiArray()
        mean_root_squared_vehicle_pose_error = Float32MultiArray()
        mean_vehicle_pose_error.layout.dim = [MultiArrayDimension()]
        mean_squared_vehicle_pose_error.layout.dim = [MultiArrayDimension()]
        mean_root_squared_vehicle_pose_error.layout.dim = [MultiArrayDimension()]
        mean_vehicle_pose_error.layout.dim[0].size = 3
        mean_squared_vehicle_pose_error.layout.dim[0].size = 3
        mean_root_squared_vehicle_pose_error.layout.dim[0].size = 3
        mean_vehicle_pose_error.data = [0.0] * 3
        mean_squared_vehicle_pose_error.data = [0.0] * 3
        mean_root_squared_vehicle_pose_error.data = [0.0] * 3
        self._pose_count_ += 1
        for i in range(3):
            self._sum_vehicle_pose_error.data[i] += vehicle_pose_error.data[i]
            self._sum_squared_vehicle_pose_error.data[i] += (
                vehicle_pose_error.data[i] ** 2
            )
            mean_vehicle_pose_error.data[i] = (
                self._sum_vehicle_pose_error.data[i] / self._pose_count_
            )
            mean_squared_vehicle_pose_error.data[i] = (
                self._sum_squared_vehicle_pose_error.data[i] / self._pose_count_
            )
            mean_root_squared_vehicle_pose_error.data[i] = (
                sqrt(self._sum_squared_vehicle_pose_error.data[i]) / self._pose_count_
            )

        # Publish vehicle state errors over time
        self._state_estimation_mean_vehicle_pose_error.publish(mean_vehicle_pose_error)
        self._state_estimation_mean_squared_vehicle_pose_error.publish(
            mean_squared_vehicle_pose_error
        )
        self._state_estimation_root_mean_squared_vehicle_pose_error.publish(
            mean_root_squared_vehicle_pose_error
        )

        # For exporting metrics to csv
        metrics = {
            "timestamp": datetime.datetime.now(),
            "vehicle_pose_error_x": vehicle_pose_error.data[0],
            "vehicle_pose_error_y": vehicle_pose_error.data[1],
            "vehicle_pose_error_theta": vehicle_pose_error.data[2],
            "mean_vehicle_pose_error_x": mean_vehicle_pose_error.data[0],
            "mean_vehicle_pose_error_y": mean_vehicle_pose_error.data[1],
            "mean_vehicle_pose_error_theta": mean_vehicle_pose_error.data[2],
            "mean_squared_vehicle_pose_error_x": mean_squared_vehicle_pose_error.data[
                0
            ],
            "mean_squared_vehicle_pose_error_y": mean_squared_vehicle_pose_error.data[
                1
            ],
            "mean_squared_vehicle_pose_error_theta": mean_squared_vehicle_pose_error.data[
                2
            ],
            "mean_root_squared_vehicle_pose_error_x": mean_root_squared_vehicle_pose_error.data[
                0
            ],
            "mean_root_squared_vehicle_pose_error_y": mean_root_squared_vehicle_pose_error.data[
                1
            ],
            "mean_root_squared_vehicle_pose_error_theta": mean_root_squared_vehicle_pose_error.data[
                2
            ],
        }
        self.pose_metrics.append(metrics)

    def compute_and_publish_map(
        self,
        map: np.ndarray,
        groundtruth_map: np.ndarray,
    ) -> None:
        """!
        Computes state estimation metrics and publishes them.

        Args:
            pose (np.ndarray): Vehicle state estimation data. [x,y,theta]
            groundtruth_pose (np.ndarray): Ground truth vehicle state data. [x,y,theta]
            map (np.ndarray): Map data. [[x,y,color,confidence]]
            groundtruth_map (np.ndarray): Ground truth map data. [[x,y,color,confidence]]
        """
        if map.size == 0 or groundtruth_map.size == 0:
            return

        self.get_logger().debug("Received map")

        # Compute instantaneous map metrics
        cone_positions = map[:, :2]
        groundtruth_cone_positions = groundtruth_map[:, :2]
        mean_difference = Float32()
        mean_difference.data = get_average_difference(
            cone_positions, groundtruth_cone_positions
        )

        mean_squared_difference = Float32()
        mean_squared_difference.data = get_mean_squared_difference(
            cone_positions, groundtruth_cone_positions
        )

        num_duplicates = Int32()
        num_duplicates.data = int(get_duplicates(cone_positions, 0.1))

        root_mean_squared_difference = Float32()
        root_mean_squared_difference.data = sqrt(
            get_mean_squared_difference(cone_positions, groundtruth_cone_positions)
        )

        false_positives = Int32()
        false_positives.data = int(
            get_false_positives(cone_positions, groundtruth_cone_positions, 2)
        )

        difference_with_map = Int32()
        difference_with_map.data = cone_positions.size - groundtruth_cone_positions.size

        self.get_logger().debug(
            "Computed slam metrics:\n \
                                Mean difference: {}\n \
                                Mean squared difference: {}\n \
                                Root mean squared difference: {}".format(
                mean_difference,
                mean_squared_difference,
                root_mean_squared_difference,
            )
        )

        # Publishes computed state estimation metrics
        self._map_mean_difference_.publish(mean_difference)
        self._map_mean_squared_difference_.publish(mean_squared_difference)
        self._map_root_mean_squared_difference_.publish(root_mean_squared_difference)
        self._se_number_duplicates.publish(num_duplicates)
        self._se_false_positives_.publish(false_positives)
        self._se_difference_with_map_.publish(difference_with_map)

        # Compute map metrics over time
        self._se_map_sum_error += get_average_difference(
            cone_positions, groundtruth_cone_positions
        )
        self._se_map_squared_sum_error += get_mean_squared_difference(
            cone_positions, groundtruth_cone_positions
        )
        self._se_map_mean_root_squared_sum_error += get_mean_squared_difference(
            cone_positions, groundtruth_cone_positions
        ) ** (1 / 2)
        self._map_count_ += 1
        mean_mean_error = Float32()
        mean_mean_error.data = self._se_map_sum_error / self._map_count_
        mean_mean_squared_error = Float32()
        mean_mean_squared_error.data = self._se_map_squared_sum_error / self._map_count_
        mean_mean_root_squared_error = Float32()
        mean_mean_root_squared_error.data = (
            self._se_map_mean_root_squared_sum_error / self._map_count_
        )

        # Publish map metrics over time
        self._map_mean_mean_error.publish(mean_mean_error)
        self._map_mean_mean_squared_error.publish(mean_mean_squared_error)
        self._map_mean_mean_root_squared_error.publish(mean_mean_root_squared_error)

        # For exporting metrics to csv
        metrics = {
            "timestamp": datetime.datetime.now(),
            "mean_difference": mean_difference.data,
            "mean_squared_difference": mean_squared_difference.data,
            "root_mean_squared_difference": root_mean_squared_difference.data,
            "mean_mean_error": mean_mean_error.data,
            "mean_mean_squared_error": mean_mean_squared_error.data,
            "mean_mean_root_squared_error": mean_mean_root_squared_error.data,
            "false_positives": false_positives.data,
            "difference_with_map": difference_with_map.data,
            "num_duplicates": num_duplicates.data,
        }
        self.map_metrics.append(metrics)

    def compute_and_publish_velocities(
        self,
        velocities: np.ndarray,
        groundtruth_velocities: np.ndarray,
    ) -> None:
        """!
        Computes state estimation metrics and publishes them.

        Args:
            velocities (np.ndarray): Vehicle state velocities. [vx, vy, w]
            groundtruth_velocities (np.ndarray): Ground truth vehicle state velocities. [vx, vy, w]
        """

        self.get_logger().debug("Received vehicle speed estimation")

        # Compute instantaneous vehicle state metrics
        velocities_error = Float32MultiArray()
        velocities_error.layout.dim = [MultiArrayDimension()]
        velocities_error.layout.dim[0].size = 3
        velocities_error.layout.dim[0].label = "vehicle state error: [vx, vy, w]"
        velocities_error.data = [0.0] * 3
        if groundtruth_velocities != []:
            velocities_error.data[0] = abs(velocities[0] - groundtruth_velocities[0])
            velocities_error.data[1] = abs(velocities[1] - groundtruth_velocities[1])
            velocities_error.data[2] = abs(velocities[2] - groundtruth_velocities[2])

        self.get_logger().debug(
            "Computed state estimation metrics:\n \
                                Vehicle velocities error: {}".format(
                velocities_error,
            )
        )

        # Publishes computed state estimation metrics
        self._velocities_difference_.publish(velocities_error)

        # Compute vehicle velocity metrics over time
        self._ve_count_ += 1
        mean_velocities_error = Float32MultiArray()
        mean_squared_velocities_error = Float32MultiArray()
        mean_root_squared_velocities_error = Float32MultiArray()
        mean_velocities_error.layout.dim = [MultiArrayDimension()]
        mean_squared_velocities_error.layout.dim = [MultiArrayDimension()]
        mean_root_squared_velocities_error.layout.dim = [MultiArrayDimension()]
        mean_velocities_error.layout.dim[0].size = 3
        mean_squared_velocities_error.layout.dim[0].size = 3
        mean_root_squared_velocities_error.layout.dim[0].size = 3
        mean_velocities_error.data = [0.0] * 3
        mean_squared_velocities_error.data = [0.0] * 3
        mean_root_squared_velocities_error.data = [0.0] * 3
        for i in range(3):
            self._sum_velocities_error.data[i] += velocities_error.data[i]
            self._sum_squared_velocities_error.data[i] += velocities_error.data[i] ** 2
            mean_velocities_error.data[i] = (
                self._sum_velocities_error.data[i] / self._ve_count_
            )
            mean_squared_velocities_error.data[i] = (
                self._sum_squared_velocities_error.data[i] / self._ve_count_
            )
            mean_root_squared_velocities_error.data[i] = (
                sqrt(self._sum_squared_velocities_error.data[i]) / self._ve_count_
            )

        # Publish vehicle state errors over time
        self._state_estimation_mean_velocities_error.publish(mean_velocities_error)
        self._state_estimation_mean_squared_velocities_error.publish(
            mean_squared_velocities_error
        )
        self._state_estimation_root_mean_squared_velocities_error.publish(
            mean_root_squared_velocities_error
        )

        # For exporting metrics to csv
        metrics = {
            "timestamp": datetime.datetime.now(),
            "velocities_error_v1": velocities_error.data[0],
            "velocities_error_v2": velocities_error.data[1],
            "velocities_error_w": velocities_error.data[2],
            "mean_velocities_error_v1": mean_velocities_error.data[0],
            "mean_velocities_error_v2": mean_velocities_error.data[1],
            "mean_velocities_error_w": mean_velocities_error.data[2],
            "mean_squared_velocities_error_v1": mean_squared_velocities_error.data[0],
            "mean_squared_velocities_error_v2": mean_squared_velocities_error.data[1],
            "mean_squared_velocities_error_w": mean_squared_velocities_error.data[2],
            "mean_root_squared_velocities_error_v1": mean_root_squared_velocities_error.data[
                0
            ],
            "mean_root_squared_velocities_error_v2": mean_root_squared_velocities_error.data[
                1
            ],
            "mean_root_squared_velocities_error_w": mean_root_squared_velocities_error.data[
                2
            ],
        }
        self.vel_estimation_metrics.append(metrics)

    def compute_and_publish_perception(
        self, perception_output: np.ndarray, perception_ground_truth: np.ndarray
    ) -> None:
        """!
        Computes perception metrics and publishes them.

        Args:
            perception_output (np.ndarray): Perceived cones.
            perception_ground_truth (np.ndarray): Ground truth cones.
        """

        if len(perception_output) == 0 or len(perception_ground_truth) == 0:
            self.get_logger().debug("Perception, ground truth or cones info missing")
            return

        # Compute instantaneous perception metrics

        cone_positions = perception_output[:, :2]
        groundtruth_cone_positions = perception_ground_truth[:, :2]
        mean_difference = Float32()
        mean_difference.data = get_average_difference(
            cone_positions, groundtruth_cone_positions
        )
        mean_squared_error = Float32()
        mean_squared_error.data = get_mean_squared_difference(
            cone_positions, groundtruth_cone_positions
        )
        inter_cones_distance = Float32()
        inter_cones_distance.data = float(get_inter_cones_distance(cone_positions))
        root_mean_squared_difference = Float32()
        root_mean_squared_difference.data = sqrt(mean_squared_error.data)

        false_positives = Int32()
        false_positives.data = int(
            get_false_positives(cone_positions, groundtruth_cone_positions, 0.1)
        )
        precision = Float32()
        precision.data = float(
            (len(cone_positions) - false_positives.data) / len(cone_positions)
        )
        recall = Float32()
        recall.data = float(
            (len(cone_positions) - false_positives.data)
            / len(groundtruth_cone_positions)
        )

        num_duplicates = Int32()
        num_duplicates.data = int(get_duplicates(perception_output, 0.1))

        self.get_logger().debug(
            "Computed perception metrics:\n \
                               Mean difference: {}\n \
                               Inter cones distance: {}\n \
                               Mean squared difference: {}\n \
                               Root mean squared difference: {}\n \
                               False positives: {}\n \
                               Precision: {}\n \
                               Recall: {}\n \
                               Duplicates: {}".format(
                mean_difference,
                inter_cones_distance,
                mean_squared_error,
                root_mean_squared_difference,
                false_positives,
                precision,
                recall,
                num_duplicates,
            )
        )

        # Publishes computed perception metrics
        self._perception_mean_difference_.publish(mean_difference)
        self._perception_inter_cones_distance_.publish(inter_cones_distance)
        self._perception_mean_squared_difference_.publish(mean_squared_error)
        self._perception_root_mean_squared_difference_.publish(
            root_mean_squared_difference
        )
        self._perception_false_positives_.publish(false_positives)
        self._perception_precision_.publish(precision)
        self._perception_recall_.publish(recall)
        self._perception_number_duplicates.publish(num_duplicates)

        # Compute perception metrics over time
        self._perception_sum_error += get_average_difference(
            cone_positions, groundtruth_cone_positions
        )
        self._perception_squared_sum_error += get_mean_squared_difference(
            cone_positions, groundtruth_cone_positions
        )
        self._perception_root_squared_sum_error += get_mean_squared_difference(
            cone_positions, groundtruth_cone_positions
        ) ** (1 / 2)
        self._perception_precision_sum += float(
            (
                len(cone_positions)
                - get_false_positives(cone_positions, groundtruth_cone_positions, 0.1)
            )
            / len(cone_positions)
        )
        self._perception_recall_sum += float(
            (
                len(cone_positions)
                - get_false_positives(cone_positions, groundtruth_cone_positions, 0.1)
            )
            / len(groundtruth_cone_positions)
        )
        self._perception_count += 1
        mean_mean_error = Float32()
        mean_mean_error.data = self._perception_sum_error / self._perception_count
        mean_mean_squared_error = Float32()
        mean_mean_squared_error.data = (
            self._perception_squared_sum_error / self._perception_count
        )
        mean_mean_root_squared_error = Float32()
        mean_mean_root_squared_error.data = (
            self._perception_root_squared_sum_error / self._perception_count
        )
        mean_precision = Float32()
        mean_precision.data = self._perception_precision_sum / self._perception_count
        mean_recall = Float32()
        mean_recall.data = self._perception_recall_sum / self._perception_count

        # Publish perception metrics over time
        self._perception_mean_mean_error.publish(mean_mean_error)
        self._perception_mean_mean_squared_error.publish(mean_mean_squared_error)
        self._perception_mean_mean_root_squared_error.publish(
            mean_mean_root_squared_error
        )
        self._perception_mean_precision.publish(mean_precision)
        self._perception_mean_recall.publish(mean_recall)

        # For exporting metrics to csv
        metrics = {
            "timestamp": datetime.datetime.now(),
            "mean_difference": mean_difference.data,
            "inter_cones_distance": inter_cones_distance.data,
            "mean_squared_difference": mean_squared_error.data,
            "root_mean_squared_difference": root_mean_squared_difference.data,
            "mean_mean_error": mean_mean_error.data,
            "mean_mean_squared_error": mean_mean_squared_error.data,
            "mean_mean_root_squared_error": mean_mean_root_squared_error.data,
            "false_positives": false_positives.data,
            "precision": precision.data,
            "recall": recall.data,
            "mean_precision": mean_precision.data,
            "mean_recall": mean_recall.data,
            "duplicates": num_duplicates.data,
        }
        self.perception_metrics.append(metrics)

    def compute_and_publish_planning(
        self,
        path: np.ndarray,
        path_gt: np.ndarray,
        left_cones_gt: np.ndarray,
        right_cones_gt: np.ndarray,
    ):
        """!
        Computes planning metrics and publishes them.

        Args:
            path (np.ndarray): Path computed by the planner.
            left_cones_gt (np.ndarray): Ground truth of the left cones.
            right_cones_gt (np.ndarray): Ground truth of the right cones.
        """
        self.get_logger().debug("Received planning")

        # Compute instantaneous planning metrics

        if (
            len(path) == 0
            or len(path_gt) == 0
            or len(left_cones_gt) == 0
            or len(right_cones_gt) == 0
        ):
            self.get_logger().debug(
                "Path, path ground truth or map ground truth info missing"
            )
            return

        # Metric 1: compute distances between cones and closes point in the path
        blue_cones = left_cones_gt
        yellow_cones = right_cones_gt

        useful_blue_cones = find_closest_elements(path, blue_cones)
        useful_yellow_cones = find_closest_elements(path, yellow_cones)
        distance_to_cones_left = compute_closest_distances(path, useful_blue_cones)
        distance_to_cones_right = compute_closest_distances(path, useful_yellow_cones)
        general_distance_to_cones = np.concatenate(
            [distance_to_cones_left, distance_to_cones_right]
        )

        mean_cones_difference_left = get_average_error(distance_to_cones_left)
        mean_squared_cones_difference_left = get_mean_squared_error(
            distance_to_cones_left
        )
        root_mean_squared_cones_difference_left = (
            mean_squared_cones_difference_left ** (1 / 2)
        )
        mean_cones_difference_right = get_average_error(distance_to_cones_right)
        mean_squared_cones_difference_right = get_mean_squared_error(
            distance_to_cones_right
        )
        root_mean_squared_cones_difference_right = (
            mean_squared_cones_difference_right ** (1 / 2)
        )
        mean_cones_difference = get_average_error(general_distance_to_cones)
        # if mean_cones_difference > 4.0:
        #    self.get_logger().info(
        #        "blue_cones: {}\n yellow_cones: {}\n useful_blue_cones: {}\n useful_yellow_cones: {}\n path: {}".format(
        #            blue_cones,
        #            yellow_cones,
        #            useful_blue_cones,
        #            useful_yellow_cones,
        #            path,
        #        )
        #    )
        mean_squared_cones_difference = get_mean_squared_error(
            general_distance_to_cones
        )
        root_mean_squared_cones_difference = mean_squared_cones_difference ** (1 / 2)
        # Metric 2: compute distance between ground truth and closest point in the path
        useful_gt_points = find_closest_elements(path, path_gt)
        distance_to_gt = compute_closest_distances(path, useful_gt_points)

        mean_gt_difference = get_average_error(distance_to_gt)
        mean_squared_gt_difference = get_mean_squared_error(distance_to_gt)
        root_mean_squared_gt_difference = mean_squared_gt_difference ** (1 / 2)

        # Log some computed metrics
        self.get_logger().debug(
            "Computed planning metrics:\n \
                                Mean difference to cones: {}\n \
                                Mean squared difference to cones: {}\n \
                                Mean difference to ground truth: {}\n \
                                Mean squared difference to ground truth: {}".format(
                mean_cones_difference,
                mean_squared_cones_difference,
                mean_gt_difference,
                mean_squared_gt_difference,
            )
        )

        # Publish planning metrics
        self._planning_mean_difference_to_gt.publish(Float32(data=mean_gt_difference))
        self._planning_mean_squared_difference_to_gt.publish(
            Float32(data=mean_squared_gt_difference)
        )
        self._planning_root_mean_squared_difference_to_gt.publish(
            Float32(data=root_mean_squared_gt_difference)
        )

        self._planning_mean_difference_to_left_cones.publish(
            Float32(data=mean_cones_difference_left)
        )
        self._planning_mean_squared_difference_to_left_cones.publish(
            Float32(data=mean_squared_cones_difference_left)
        )
        self._planning_root_mean_squared_difference_to_left_cones.publish(
            Float32(data=root_mean_squared_cones_difference_left)
        )

        self._planning_mean_difference_to_right_cones.publish(
            Float32(data=mean_cones_difference_right)
        )
        self._planning_mean_squared_difference_to_right_cones.publish(
            Float32(data=mean_squared_cones_difference_right)
        )
        self._planning_root_mean_squared_difference_to_right_cones.publish(
            Float32(data=root_mean_squared_cones_difference_right)
        )

        self._planning_mean_difference_to_cones.publish(
            Float32(data=mean_cones_difference)
        )
        self._planning_mean_squared_difference_to_cones.publish(
            Float32(data=mean_squared_cones_difference)
        )
        self._planning_root_mean_squared_difference_to_cones.publish(
            Float32(data=root_mean_squared_cones_difference)
        )

        # Compute planning metrics over time
        # self._planning_sum_error += get_average_difference(path, path_gt)
        # self._planning_squared_sum_error += get_mean_squared_difference(path, path_gt)
        # self._planning_mean_root_squared_sum_error += get_mean_squared_difference(
        #    path, path_gt
        # ) ** (1 / 2)
        # self._planning_count += 1
        # mean_mean_error = Float32()
        # mean_mean_error.data = self._planning_sum_error / self._planning_count
        # mean_mean_squared_error = Float32()
        # mean_mean_squared_error.data = (
        #    self._planning_squared_sum_error / self._planning_count
        # )
        # mean_mean_root_squared_error = Float32()
        # mean_mean_root_squared_error.data = (
        #    self._planning_mean_root_squared_sum_error / self._planning_count
        # )

        # Publish planning metrics over time
        # self._planning_mean_mean_error.publish(mean_mean_error)
        # self._planning_mean_mean_squared_error.publish(mean_mean_squared_error)
        # self._planning_mean_mean_root_squared_error.publish(
        #    mean_mean_root_squared_error
        # )

        # For exporting metrics to csv
        metrics = {
            "timestamp": datetime.datetime.now(),
            "mean_difference_to_gt": mean_gt_difference.data,
            "mean_squared_difference_to_gt": mean_squared_gt_difference.data,
            "root_mean_squared_difference_to_gt": root_mean_squared_gt_difference.data,
            "mean_difference_left_cones": mean_cones_difference_left.data,
            "mean_squared_difference_left_cones": mean_squared_cones_difference_left.data,
            "root_mean_squared_difference_left_cones": root_mean_squared_cones_difference_left.data,
            "mean_difference_right_cones": mean_cones_difference_right.data,
            "mean_squared_difference_right_cones": mean_squared_cones_difference_right.data,
            "root_mean_squared_difference_right_cones": root_mean_squared_cones_difference_right.data,
            "mean_difference_to_cones": mean_cones_difference.data,
            "mean_squared_difference_to_cones": mean_squared_cones_difference.data,
            "root_mean_squared_difference_to_cones": root_mean_squared_cones_difference.data,
        }
        self.planning_metrics.append(metrics)

    def compute_and_publish_control(self, msg: EvaluatorControlData):
        """!
        Computes control metrics and publishes them.
        Args:
            msg (EvaluatorControlData): Control info data message.
        """
        self.get_logger().debug("Received control")

        # Compute instantaneous control metrics
        pose_treated, velocities_treated = format_vehicle_state_msg(msg.vehicle_state)
        lookahead_velocity = msg.lookahead_velocity

        pose_position = pose_treated[:2]
        closest_point = format_point2d_msg(msg.closest_point)

        pose_difference = Float32()
        pose_difference.data = compute_distance(closest_point, pose_position)
        velocity_to_lookahead_velocity_difference = Float32()
        velocity_to_lookahead_velocity_difference.data = float(
            velocities_treated[0] - lookahead_velocity
        )

        velocity_to_closest_difference = Float32()
        velocity_to_closest_difference.data = float(
            velocities_treated[0] - msg.closest_point_velocity
        )

        self.get_logger().debug(
            "Computed control metrics:\n \
                                Pose Difference: {}\n \
                                Velocity Difference: {}\n \
                                Lookahead Velocity Difference: {}".format(
                pose_difference,
                velocity_to_closest_difference,
                velocity_to_lookahead_velocity_difference,
            )
        )

        # Publish control metrics
        self._control_closest_pose_difference_.publish(pose_difference)
        self._control_velocity_closest_difference_.publish(
            velocity_to_closest_difference
        )
        self._control_velocity_lookahead_difference_.publish(
            velocity_to_lookahead_velocity_difference
        )

        # Compute control metrics over time
        self._control_pose_sum_error += pose_difference.data
        self._control_pose_squared_sum_error += pose_difference.data**2
        self._control_velocity_sum_error += (
            velocity_to_lookahead_velocity_difference.data
        )
        self._control_velocity_squared_sum_error += (
            velocity_to_lookahead_velocity_difference.data**2
        )
        self._control_count += 1
        self._control_closest_velocity_sum_error += float(
            velocities_treated[0] - msg.closest_point_velocity
        )
        self.control_closest_velocity_squared_sum_error += (
            float(velocities_treated[0] - msg.closest_point_velocity) ** 2
        )

        velocity_to_lookahead_mean_difference = Float32()
        velocity_to_lookahead_mean_difference.data = float(
            self._control_velocity_sum_error / self._control_count
        )

        velocity_to_lookahead_mean_squared_difference = Float32()
        velocity_to_lookahead_mean_squared_difference.data = float(
            self._control_velocity_squared_sum_error / self._control_count
        )

        velocity_to_lookahead_root_mean_squared = Float32()
        velocity_to_lookahead_root_mean_squared.data = sqrt(
            self._control_velocity_squared_sum_error / self._control_count
        )

        velocity_to_closest_mean_difference = Float32()
        velocity_to_closest_mean_difference.data = float(
            self._control_closest_velocity_sum_error / self._control_count
        )

        velocity_to_closest_mean_squared_difference = Float32()
        velocity_to_closest_mean_squared_difference.data = float(
            self.control_closest_velocity_squared_sum_error / self._control_count
        )

        velocity_to_closest_root_mean_squared = Float32()
        velocity_to_closest_root_mean_squared.data = sqrt(
            self.control_closest_velocity_squared_sum_error / self._control_count
        )

        pose_mean_difference = Float32()
        pose_mean_difference.data = float(
            self._control_pose_sum_error / self._control_count
        )

        pose_mean_squared_difference = Float32()
        pose_mean_squared_difference.data = float(
            self._control_pose_squared_sum_error / self._control_count
        )

        pose_root_mean_squared_difference = Float32()
        pose_root_mean_squared_difference.data = sqrt(
            self._control_pose_squared_sum_error / self._control_count
        )

        # Publish control metrics over time
        self._control_pose_difference_mean_.publish(pose_mean_difference)
        self._control_pose_mean_squared_difference_.publish(
            pose_mean_squared_difference
        )
        self._control_pose_root_mean_squared_difference_.publish(
            pose_root_mean_squared_difference
        )

        self._control_velocity_to_lookahead_velocity_difference_mean_.publish(
            velocity_to_lookahead_mean_difference
        )
        self._control_pose_mean_squared_difference_.publish(
            velocity_to_lookahead_mean_squared_difference
        )
        self._control_velocity_to_lookahead_velocity_mean_squared_difference_.publish(
            velocity_to_lookahead_root_mean_squared
        )
        self._control_velocity_to_closest_velocity_difference_mean_.publish(
            velocity_to_closest_mean_difference
        )
        self._control_velocity_to_closest_velocity_mean_squared_difference_.publish(
            velocity_to_closest_mean_squared_difference
        )
        self._control_velocity_to_closest_velocity_root_mean_squared_difference_.publish(
            velocity_to_closest_root_mean_squared
        )

        self._control_execution_time_.append(
            {"timestamp": datetime.datetime.now(), "execution_time": msg.execution_time}
        )

        # For exporting metrics to csv
        metrics = {
            "timestamp": datetime.datetime.now(),
            "difference": pose_difference.data,
            "mean_difference": pose_mean_difference.data,
            "mean_squared_difference": pose_mean_squared_difference.data,
            "root_mean_squared_difference": pose_root_mean_squared_difference.data,
            "velocity_difference": velocity_to_lookahead_velocity_difference.data,
            "velocity_mean_difference": velocity_to_lookahead_mean_difference.data,
            "velocity_mean_squared_difference": velocity_to_lookahead_mean_squared_difference.data,
            "velocity_root_mean_squared_difference": velocity_to_lookahead_root_mean_squared.data,
            "velocity_to_closest_difference": velocity_to_closest_difference.data,
            "velocity_to_closest_mean_difference": velocity_to_closest_mean_difference.data,
            "velocity_to_closest_mean_squared_difference": velocity_to_closest_mean_squared_difference.data,
            "velocity_to_closest_root_mean_squared_difference": velocity_to_closest_root_mean_squared.data,
        }
        self.control_metrics.append(metrics)


def main(args=None):
    """
    Main function for the evaluator module.
    Args:
        args (list): Optional command-line arguments.
    """
    rclpy.init(args=args)
    node = Evaluator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
