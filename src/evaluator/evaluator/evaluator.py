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
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension, Int32
from sensor_msgs.msg import PointCloud2
import datetime
from math import sqrt
from evaluator.formats import format_path_point_array_msg
from message_filters import TimeSynchronizer

import csv
import signal


class Evaluator(Node):
    """!
    A ROS2 node for computing and publishing system's metrics
    """

    def __init__(self):
        """!
        Initializes the Evaluator node and creates the subscriptions/adapters
        """
        super().__init__("evaluator")
        self.get_logger().info("Evaluator Node has started")

        # Parameters
        self._adapter_name_: str = (
            self.declare_parameter("adapter", "vehicle")
            .get_parameter_value()
            .string_value
        )
        self.use_simulated_perception_: bool = (
            self.declare_parameter("use_simulated_perception", False)
            .get_parameter_value()
            .bool_value
        )
        self.use_simulated_se_: bool = (
            self.declare_parameter("use_simulated_se", False)
            .get_parameter_value()
            .bool_value
        )
        self.use_simulated_planning_: bool = (
            self.declare_parameter("use_simulated_planning", False)
            .get_parameter_value()
            .bool_value
        )
        if (self._adapter_name_ == "fsds") and (self.use_simulated_perception_):
            self.get_logger().error(
                "Simulated perception is not supported for FSDS adapter"
            )
            sys.exit(1)

        self._point_cloud_receive_time_: datetime.datetime = datetime.datetime.now()
        self.perception_receive_time_: datetime.datetime = datetime.datetime.now()
        self.map_receive_time_: datetime.datetime = datetime.datetime.now()
        self._planning_receive_time_: datetime.datetime = datetime.datetime.now()
        self._control_receive_time_: datetime.datetime = datetime.datetime.now()

        # Subscriptions
        self._perception_timing_subscription_ = self.create_subscription(
            ConeArray,
            "/perception/cones",
            self.perception_callback_time_measurement,
            10,
        )
        self.perception_subscription_ = message_filters.Subscriber(
            self, ConeArray, "/perception/cones"
        )
        self._map_timing_subscription_ = self.create_subscription(
            ConeArray,
            "/state_estimation/map",
            self.map_callback_time_measurement,
            10,
        )
        self.map_subscription_ = message_filters.Subscriber(
            self, ConeArray, "/state_estimation/map"
        )
        self.vehicle_state_subscription_ = message_filters.Subscriber(
            self, VehicleState, "/state_estimation/vehicle_state"
        )
        self.transform_buffer_ = tf2_ros.Buffer()
        self._transforms_listener_ = TransformListener(self.transform_buffer_, self)
        self.point_cloud_timing_subscription_ = self.create_subscription(
            PointCloud2,
            ADAPTER_POINT_CLOUD_TOPIC_DICTIONARY[self._adapter_name_],
            self.point_cloud_callback,
            ADAPTER_POINT_CLOUD_TOPIC_QOS_DICTIONARY[self._adapter_name_],
        )
        self.planning_subscription = self.create_subscription(
            PathPointArray, "/path_planning/path", self.compute_and_publish_planning, 1
        )
        self.planning_gt_subscription = self.create_subscription(
            PathPointArray, "path_planning/mock_path", self.planning_gt_callback, 1
        )
        self.control_data_sub_ = self.create_subscription(
            EvaluatorControlData,
            "control/evaluator_data",
            self.compute_and_publish_control,
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
        self._perception_execution_time_ = self.create_publisher(
            Float32, "/evaluator/perception/execution_time", 10
        )

        # Publishers for state estimation metrics
        self._map_execution_time_ = self.create_publisher(
            Float32, "/evaluator/state_estimation/execution_time", 10
        )
        self._vehicle_state_difference_ = self.create_publisher(
            Float32MultiArray,
            "/evaluator/state_estimation/vehicle_state_difference",
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

        self._se_difference_with_map_ = self.create_publisher(
            Int32, "/evaluator/state_estimation/difference_with_map", 10
        )

        # Publisher for path planning metrics
        self._planning_mean_difference_ = self.create_publisher(
            Float32, "/evaluator/planning/mean_difference", 10
        )
        self._planning_mean_squared_difference_ = self.create_publisher(
            Float32, "/evaluator/planning/mean_squared_difference", 10
        )
        self._planning_root_mean_squared_difference_ = self.create_publisher(
            Float32, "/evaluator/planning/root_mean_squared_difference", 10
        )
        self._planning_execution_time_ = self.create_publisher(
            Float32, "/evaluator/planning/execution_time", 10
        )

        # Publisehr for control metrics
        self._control_execution_time_ = self.create_publisher(
            Float32, "/evaluator/control/execution_time", 10
        )
        self._control_pose_difference_ = self.create_publisher(
            Float32, "/evaluator/control/pose/difference", 10
        )

        self._control_velocity_lookahead_difference_ = self.create_publisher(
            Float32, "/evaluator/control/velocity/difference", 10
        )

        # Metrics over time
        self.perception_metrics = []
        self.se_metrics = []
        self.planning_metrics = []
        self.control_metrics = []

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
        self._perception_count = 0

        self._se_map_sum_error = 0
        self._se_map_squared_sum_error = 0
        self._se_map_mean_root_squared_sum_error = 0
        self._se_count = 0

        self._sum_vehicle_state_error = Float32MultiArray()
        self._sum_squared_vehicle_state_error = Float32MultiArray()
        self._sum_vehicle_state_error.layout.dim = [MultiArrayDimension()]
        self._sum_squared_vehicle_state_error.layout.dim = [MultiArrayDimension()]
        self._sum_vehicle_state_error.layout.dim[0].size = 6
        self._sum_squared_vehicle_state_error.layout.dim[0].size = 6
        self._sum_vehicle_state_error.data = [0.0] * 6
        self._sum_squared_vehicle_state_error.data = [0.0] * 6
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

        self._state_estimation_mean_vehicle_state_error = self.create_publisher(
            Float32MultiArray,
            "/evaluator/state_estimation/vehicle_state_mean_difference",
            10,
        )

        self._state_estimation_mean_squared_vehicle_state_error = self.create_publisher(
            Float32MultiArray,
            "/evaluator/state_estimation/vehicle_state_mean_squared_difference",
            10,
        )

        self._state_estimation_root_mean_squared_vehicle_state_error = self.create_publisher(
            Float32MultiArray,
            "/evaluator/state_estimation/vehicle_state_mean_root_squared_difference",
            10,
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

        self._control_velocity_to_closest_velocity_difference_ = self.create_publisher(
            Float32, "/evaluator/control/velocity/closest/difference", 10
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

        self.planning_mock = PathPointArray()

        if self._adapter_name_ == "vehicle":
            return

        # Adapter selection
        self._adapter_: Adapter = ADAPTER_CONSTRUCTOR_DICTINARY[self._adapter_name_](
            self
        )

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        """!
        Writes metrics to csv and exits when Ctrl+C is pressed.

        Args:
            sig (int): Signal number.
            frame (frame): Current stack frame.
        """

        finish_time = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        metrics_dict = {
            "perception": self.perception_metrics,
            "se": self.se_metrics,
            "planning": self.planning_metrics,
            "control": self.control_metrics,
        }
        for filename, metrics in metrics_dict.items():
            if metrics:
                datetime_filename = f"{filename}_{finish_time}.csv"
                self.metrics_to_csv(
                    metrics, "performance/evaluator_metrics/" + datetime_filename
                )
        sys.exit(0)

    def metrics_to_csv(self, metrics, filename):
        """!
        Converts metrics to csv and writes them to a file.

        Args:
            metrics (list): List of metrics dictionaries.
            filename (str): Name of the file to write the metrics to.
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

    def point_cloud_callback(self, _: PointCloud2):
        """!
        Point Cloud Callback to get the initial time of perception pipeline
        """
        self._point_cloud_receive_time_ = datetime.datetime.now()

    def perception_callback_time_measurement(self, _: ConeArray) -> None:
        """!
        Computes the perception's execution time
        """

        self.get_logger().debug("Received perception")
        self.perception_receive_time_ = datetime.datetime.now()
        time_difference = float(
            (
                self.perception_receive_time_ - self._point_cloud_receive_time_
            ).microseconds
            / 1000
        )
        execution_time = Float32()
        execution_time.data = time_difference
        self._perception_execution_time_.publish(execution_time)

    def map_callback_time_measurement(self, _: ConeArray) -> None:
        """!
        Computes the state estimation's execution time
        """

        self.get_logger().debug("Received map")
        self.map_receive_time_ = datetime.datetime.now()
        self.pose_receive_time_ = datetime.datetime.now()
        time_difference: datetime.timedelta = float(
            (self.map_receive_time_ - self.perception_receive_time_).microseconds / 1000
        )
        execution_time = Float32()
        execution_time.data = time_difference
        self._map_execution_time_.publish(execution_time)

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
        if map.size == 0 or groundtruth_map.size == 0:
            return

        # Compute execution time
        self.map_receive_time_ = datetime.datetime.now()
        self.pose_receive_time_ = datetime.datetime.now()
        time_difference: datetime.timedelta = float(
            (self.map_receive_time_ - self.perception_receive_time_).microseconds / 1000
        )
        execution_time = Float32()
        execution_time.data = time_difference
        self._map_execution_time_.publish(execution_time)

        # Compute instantaneous vehicle state metrics
        vehicle_state_error = Float32MultiArray()
        vehicle_state_error.layout.dim = [MultiArrayDimension()]
        vehicle_state_error.layout.dim[0].size = 6
        vehicle_state_error.layout.dim[0].label = (
            "vehicle state error: [x, y, theta, v, v, w]"
        )
        vehicle_state_error.data = [0.0] * 6
        vehicle_state_error.data[0] = abs(pose[0] - groundtruth_pose[0])
        vehicle_state_error.data[1] = abs(pose[1] - groundtruth_pose[1])
        vehicle_state_error.data[2] = abs(pose[2] - groundtruth_pose[2]) % (2 * np.pi)
        vehicle_state_error.data[3] = abs(
            velocities[0]
            - sqrt(
                pow(groundtruth_velocities[0], 2) + pow(groundtruth_velocities[1], 2)
            )
        )
        vehicle_state_error.data[4] = abs(
            velocities[1]
            - sqrt(
                pow(groundtruth_velocities[0], 2) + pow(groundtruth_velocities[1], 2)
            )
        )
        vehicle_state_error.data[5] = abs(velocities[2] - groundtruth_velocities[2])

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

        root_mean_squared_difference = Float32()
        root_mean_squared_difference.data = sqrt(
            get_mean_squared_difference(cone_positions, groundtruth_cone_positions)
        )

        self.get_logger().debug(
            "Computed state estimation metrics:\n \
                                Vehicle state error: {}\n \
                                Mean difference: {}\n \
                                Mean squared difference: {}\n \
                                Root mean squared difference: {}".format(
                vehicle_state_error,
                mean_difference,
                mean_squared_difference,
                root_mean_squared_difference,
            )
        )

        # Publishes computed state estimation metrics
        self._vehicle_state_difference_.publish(vehicle_state_error)
        self._map_mean_difference_.publish(mean_difference)
        self._map_mean_squared_difference_.publish(mean_squared_difference)
        self._map_root_mean_squared_difference_.publish(root_mean_squared_difference)

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
        self._se_count += 1

        # Compute vehicle state metrics over time
        mean_vehicle_state_error = Float32MultiArray()
        mean_squared_vehicle_state_error = Float32MultiArray()
        mean_root_squared_vehicle_state_error = Float32MultiArray()
        mean_vehicle_state_error.layout.dim = [MultiArrayDimension()]
        mean_squared_vehicle_state_error.layout.dim = [MultiArrayDimension()]
        mean_root_squared_vehicle_state_error.layout.dim = [MultiArrayDimension()]
        mean_vehicle_state_error.layout.dim[0].size = 6
        mean_squared_vehicle_state_error.layout.dim[0].size = 6
        mean_root_squared_vehicle_state_error.layout.dim[0].size = 6
        mean_vehicle_state_error.data = [0.0] * 6
        mean_squared_vehicle_state_error.data = [0.0] * 6
        mean_root_squared_vehicle_state_error.data = [0.0] * 6
        for i in range(6):
            self._sum_vehicle_state_error.data[i] += vehicle_state_error.data[i]
            self._sum_squared_vehicle_state_error.data[i] += (
                vehicle_state_error.data[i] ** 2
            )
            mean_vehicle_state_error.data[i] = (
                self._sum_vehicle_state_error.data[i] / self._se_count
            )
            mean_squared_vehicle_state_error.data[i] = (
                self._sum_squared_vehicle_state_error.data[i] / self._se_count
            )
            mean_root_squared_vehicle_state_error.data[i] = (
                sqrt(self._sum_squared_vehicle_state_error.data[i]) / self._se_count
            )

        # Publish vehicle state errors over time
        self._state_estimation_mean_vehicle_state_error.publish(
            mean_vehicle_state_error
        )
        self._state_estimation_mean_squared_vehicle_state_error.publish(
            mean_squared_vehicle_state_error
        )
        self._state_estimation_root_mean_squared_vehicle_state_error.publish(
            mean_root_squared_vehicle_state_error
        )

        # Compute map metrics over time
        mean_mean_error = Float32()
        mean_mean_error.data = self._se_map_sum_error / self._se_count
        self._map_mean_mean_error.publish(mean_mean_error)

        mean_mean_squared_error = Float32()
        mean_mean_squared_error.data = self._se_map_squared_sum_error / self._se_count
        self._map_mean_mean_squared_error.publish(mean_mean_squared_error)

        mean_mean_root_squared_error = Float32()
        mean_mean_root_squared_error.data = (
            self._se_map_mean_root_squared_sum_error / self._se_count
        )

        # Publish map metrics over time
        self._map_mean_mean_root_squared_error.publish(mean_mean_root_squared_error)
        self._map_mean_mean_error.publish(mean_mean_error)
        self._map_mean_mean_squared_error.publish(mean_mean_squared_error)

        false_positives = Int32()
        false_positives.data = get_false_positives(map, groundtruth_map, 0.1)
        self._se_false_positives_.publish(false_positives)

        difference_with_map = Int32()
        difference_with_map.data = map.size - groundtruth_map.size
        self._se_difference_with_map_.publish(difference_with_map)

        # For exporting metrics to csv
        metrics = {
            "timestamp": datetime.datetime.now(),
            "vehicle_state_error_x": vehicle_state_error.data[0],
            "vehicle_state_error_y": vehicle_state_error.data[1],
            "vehicle_state_error_theta": vehicle_state_error.data[2],
            "vehicle_state_error_v1": vehicle_state_error.data[3],
            "vehicle_state_error_v2": vehicle_state_error.data[4],
            "vehicle_state_error_w": vehicle_state_error.data[5],
            "mean_difference": mean_difference.data,
            "mean_squared_difference": mean_squared_difference.data,
            "root_mean_squared_difference": root_mean_squared_difference.data,
            "mean_vehicle_state_error_x": mean_vehicle_state_error.data[0],
            "mean_vehicle_state_error_y": mean_vehicle_state_error.data[1],
            "mean_vehicle_state_error_theta": mean_vehicle_state_error.data[2],
            "mean_vehicle_state_error_v1": mean_vehicle_state_error.data[3],
            "mean_vehicle_state_error_v2": mean_vehicle_state_error.data[4],
            "mean_vehicle_state_error_w": mean_vehicle_state_error.data[5],
            "mean_squared_vehicle_state_error_x": mean_squared_vehicle_state_error.data[
                0
            ],
            "mean_squared_vehicle_state_error_y": mean_squared_vehicle_state_error.data[
                1
            ],
            "mean_squared_vehicle_state_error_theta": mean_squared_vehicle_state_error.data[
                2
            ],
            "mean_squared_vehicle_state_error_v1": mean_squared_vehicle_state_error.data[
                3
            ],
            "mean_squared_vehicle_state_error_v2": mean_squared_vehicle_state_error.data[
                4
            ],
            "mean_squared_vehicle_state_error_w": mean_squared_vehicle_state_error.data[
                5
            ],
            "mean_root_squared_vehicle_state_error_x": mean_root_squared_vehicle_state_error.data[
                0
            ],
            "mean_root_squared_vehicle_state_error_y": mean_root_squared_vehicle_state_error.data[
                1
            ],
            "mean_root_squared_vehicle_state_error_theta": mean_root_squared_vehicle_state_error.data[
                2
            ],
            "mean_root_squared_vehicle_state_error_v1": mean_root_squared_vehicle_state_error.data[
                3
            ],
            "mean_root_squared_vehicle_state_error_v2": mean_root_squared_vehicle_state_error.data[
                4
            ],
            "mean_root_squared_vehicle_state_error_w": mean_root_squared_vehicle_state_error.data[
                5
            ],
            "mean_mean_error": mean_mean_error.data,
            "mean_mean_squared_error": mean_mean_squared_error.data,
            "mean_mean_root_squared_error": mean_mean_root_squared_error.data,
            "false_positives": false_positives.data,
            "difference_with_map": difference_with_map.data,
        }
        self.se_metrics.append(metrics)

    def compute_and_publish_perception(
        self, perception_output: np.ndarray, perception_ground_truth: np.ndarray
    ) -> None:
        """!
        Computes perception metrics and publishes them.

        Args:
            perception_output (np.ndarray): Perceived cones.
            perception_ground_truth (np.ndarray): Ground truth cones.
        """

        # Compute instantaneous perception metrics

        # TODO: include confidence in evaluator
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
        inter_cones_distance.data = get_inter_cones_distance(cone_positions)
        root_mean_squared_difference = Float32()
        root_mean_squared_difference.data = sqrt(mean_squared_error.data)

        false_positives = Int32()
        false_positives.data = get_false_positives(
            perception_output, perception_ground_truth, 0.1
        )

        self.get_logger().debug(
            "Computed perception metrics:\n \
                               Mean difference: {}\n \
                               Inter cones distance: {}\n \
                               Mean squared difference: {}\n \
                               Root mean squared difference: {}\n \
                                False positives: {}".format(
                mean_difference,
                inter_cones_distance,
                mean_squared_error,
                root_mean_squared_difference,
                false_positives,
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

        # Publish perception metrics over time
        self._perception_mean_mean_error.publish(mean_mean_error)
        self._perception_mean_mean_squared_error.publish(mean_mean_squared_error)
        self._perception_mean_mean_root_squared_error.publish(
            mean_mean_root_squared_error
        )

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
        }
        self.perception_metrics.append(metrics)

    def compute_and_publish_planning(self, msg: PathPointArray):
        """!
        Computes planning metrics and publishes them.

        Args:
            msg (PathPointArray): Path points array message.
        """
        self.get_logger().debug("Received planning")

        # Execution time metrics calculation
        self._planning_receive_time_ = datetime.datetime.now()
        time_difference = float(
            (self._planning_receive_time_ - self.map_receive_time_).microseconds / 1000
        )
        execution_time = Float32()
        execution_time.data = time_difference

        self._planning_execution_time_.publish(execution_time)

        # Compute instantaneous planning metrics
        actual_path: np.ndarray = format_path_point_array_msg(msg)
        expected_path: np.ndarray = format_path_point_array_msg(self.planning_mock)

        if len(actual_path) == 0 or len(expected_path) == 0:
            self.get_logger().debug("Path info missing")
            return

        mean_difference = Float32()
        mean_difference.data = get_average_difference(actual_path, expected_path)
        mean_squared_difference = Float32()
        mean_squared_difference.data = get_mean_squared_difference(
            actual_path, expected_path
        )
        root_mean_squared_difference = Float32()
        root_mean_squared_difference.data = sqrt(
            get_mean_squared_difference(actual_path, expected_path)
        )

        self.get_logger().debug(
            "Computed planning metrics:\n \
                                Mean difference: {}\n \
                                Mean squared difference: {}\n \
                                Root mean squared difference: {}".format(
                mean_difference,
                mean_squared_difference,
                root_mean_squared_difference,
            )
        )

        # Publish planning metrics
        self._planning_mean_difference_.publish(mean_difference)
        self._planning_mean_squared_difference_.publish(mean_squared_difference)
        self._planning_root_mean_squared_difference_.publish(
            root_mean_squared_difference
        )

        # Compute planning metrics over time
        self._planning_sum_error += get_average_difference(actual_path, expected_path)
        self._planning_squared_sum_error += get_mean_squared_difference(
            actual_path, expected_path
        )
        self._planning_mean_root_squared_sum_error += get_mean_squared_difference(
            actual_path, expected_path
        ) ** (1 / 2)
        self._planning_count += 1
        mean_mean_error = Float32()
        mean_mean_error.data = self._planning_sum_error / self._planning_count
        mean_mean_squared_error = Float32()
        mean_mean_squared_error.data = (
            self._planning_squared_sum_error / self._planning_count
        )
        mean_mean_root_squared_error = Float32()
        mean_mean_root_squared_error.data = (
            self._planning_mean_root_squared_sum_error / self._planning_count
        )

        # Publish planning metrics over time
        self._planning_mean_mean_error.publish(mean_mean_error)
        self._planning_mean_mean_squared_error.publish(mean_mean_squared_error)
        self._planning_mean_mean_root_squared_error.publish(
            mean_mean_root_squared_error
        )

        # For exporting metrics to csv
        metrics = {
            "timestamp": datetime.datetime.now(),
            "mean_difference": mean_difference.data,
            "mean_squared_difference": mean_squared_difference.data,
            "root_mean_squared_difference": root_mean_squared_difference.data,
            "mean_mean_error": mean_mean_error.data,
            "mean_mean_squared_error": mean_mean_squared_error.data,
            "mean_mean_root_squared_error": mean_mean_root_squared_error.data,
        }
        self.planning_metrics.append(metrics)

    def planning_gt_callback(self, msg: PathPointArray):
        """!
        Stores the path planning ground truth from mocker node.

        Args:
            msg (PathPointArray): Path points array message.
        """
        self.get_logger().debug("Received GT planning")
        self.planning_mock: PathPointArray = msg
        if self.use_simulated_planning_:
            self._planning_receive_time_ = datetime.datetime.now()

    def compute_and_publish_control(self, msg: EvaluatorControlData):
        """!
        Computes control metrics and publishes them.
        Args:
            vehicle_state (VehicleState): Vehicle state message.
            closest_point (PathPoint): Closest point message.
        """

        # No execution time for control because it is calculated by control node
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
        self._control_pose_difference_.publish(pose_difference)
        self._control_velocity_to_closest_velocity_difference_.publish(
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
    rclpy.init(args=args)
    node = Evaluator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
