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
    ADAPTER_POINT_CLOUD_TOPIC_DICTINARY,
)
import message_filters
import numpy as np
import rclpy.subscription
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import PointCloud2
import datetime
from math import sqrt
from evaluator.formats import format_path_point_array_msg
from message_filters import TimeSynchronizer


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
            rclpy.get_logger().error(
                "Simulated perception is not supported for FSDS adapter"
            )
            sys.exit(1)
        if (self.use_simulated_planning_ and (self._adapter_name_ == "fsds" or self._adapter_name_ == "pacsim")):
            rclpy.get_logger().error(
                "Simulated planning is not supported for FSDS and PacSIM adapter"
            )
            sys.exit(1)

        self._point_cloud_receive_time_: datetime.datetime = datetime.datetime.now()
        self.perception_receive_time_: datetime.datetime = datetime.datetime.now()
        self.map_receive_time_: datetime.datetime = datetime.datetime.now()
        self._planning_receive_time_: datetime.datetime = datetime.datetime.now()
        self._control_receive_time_: datetime.datetime = datetime.datetime.now()

        # Subscriptions
        self.perception_timing_subscription_ = self.create_subscription(
            ConeArray,
            "/perception/cones",
            self.perception_callback_time_measurement,
            10,
        )
        self.perception_subscription_ = message_filters.Subscriber(
            self, ConeArray, "/perception/cones"
        )
        self.map_timing_subscription_ = self.create_subscription(
            ConeArray, "/state_estimation/map", self.map_callback_time_measurement, 10
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
            ADAPTER_POINT_CLOUD_TOPIC_DICTINARY[self._adapter_name_],
            self.point_cloud_callback,
            10,
        )
        self.point_cloud_subscription_ = message_filters.Subscriber(
            self,
            PointCloud2,
            ADAPTER_POINT_CLOUD_TOPIC_DICTINARY[self._adapter_name_],
        )
        self.planning_subscription = self.create_subscription(
            PathPointArray, "path_planning/path", self.compute_and_publish_planning, 10
        )
        self.planning_gt_subscription = self.create_subscription(
            PathPointArray, "path_planning/mock_path", self.planning_gt_callback, 10
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

        self._control_execution_time_ = self.create_publisher(
            Float32, "/evaluator/control/execution_time", 10
        )
        self._control_difference_ = self.create_publisher(
            Float32, "/evaluator/control/pose/difference", 10
        )
        self._control_difference_mean_ = self.create_publisher(
            Float32, "/evaluator/control/pose/difference_mean", 10
        )

        self._control_mean_squared_difference_ = self.create_publisher(
            Float32, "/evaluator/control/pose/mean_squared_difference", 10
        )

        self._control_root_mean_squared_difference_ = self.create_publisher(
            Float32, "/evaluator/control/pose/root_mean_squared_difference", 10
        )

        self._control_velocity_difference_ = self.create_publisher(
            Float32, "/evaluator/control/velocity/difference", 10
        )

        self._control_velocity_difference_mean_ = self.create_publisher(
            Float32, "/evaluator/control/velocity/difference_mean", 10
        )

        self._control_velocity_mean_squared_difference_ = self.create_publisher(
            Float32, "/evaluator/control/velocity/mean_squared_difference", 10
        )

        self._control_velocity_root_mean_squared_difference_ = self.create_publisher(
            Float32, "/evaluator/control/velocity/root_mean_squared_difference", 10
        )

        self._control_sum_error = 0
        self._control_squared_sum_error = 0
        self._control_velocity_sum_error = 0
        self._control_velocity_squared_sum_error = 0
        self._control_count = 0

        self._perception_sum_error = 0
        self._perception_squared_sum_error = 0
        self._perception_root_squared_sum_error = 0
        self._perception_count = 0

        self._perception_mean_mean_error = self.create_publisher(
            Float32, "/evaluator/perception/mean_mean_error", 10
        )

        self._perception_mean_mean_squared_error = self.create_publisher(
            Float32, "/evaluator/perception/mean_mean_squared_error", 10
        )

        self._perception_mean_mean_root_squared_error = self.create_publisher(
            Float32, "/evaluator/perception/mean_mean_root_squared_error", 10
        )

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

        self._state_estimation_mean_mean_error = self.create_publisher(
            Float32, "/evaluator/state_estimation/mean_mean_error", 10
        )

        self._state_estimation_mean_mean_squared_error = self.create_publisher(
            Float32, "/evaluator/state_estimation/mean_mean_squared_error", 10
        )

        self._state_estimation_mean_mean_root_squared_error = self.create_publisher(
            Float32, "/evaluator/state_estimation/mean_mean_root_squared_error", 10
        )

        self._state_estimation_mean_mean_state_error = self.create_publisher(
            Float32MultiArray, "/evaluator/state_estimation/mean_mean_pose_and_vel_error", 10
        )

        self._state_estimation_mean_mean_state_squared_error = self.create_publisher(
            Float32MultiArray, "/evaluator/state_estimation/mean_mean_pose_and_vel_squared_error", 10
        )

        self._state_estimation_mean_mean_state_root_squared_error = self.create_publisher(
            Float32MultiArray, "/evaluator/state_estimation/mean_mean_root_pose_and_vel_squared_error", 10
        )

        self._planning_sum_error = 0
        self._planning_squared_sum_error = 0
        self._planning_mean_root_squared_sum_error = 0
        self._planning_count = 0
        
        self._planning_mean_mean_error = self.create_publisher(
            Float32, "/evaluator/planning/mean_mean_error", 10
        )

        self._planning_mean_mean_squared_error = self.create_publisher(
            Float32, "/evaluator/planning/mean_mean_squared_error", 10
        )

        self._planning_mean_mean_root_squared_error = self.create_publisher(
            Float32, "/evaluator/planning/mean_mean_root_squared_error", 10
        )


        self.planning_mock = (
            []
        )  # will store the reception of a planning mock from subscriber

        if self._adapter_name_ == "vehicle":
            return

        # Adapter selection
        self._adapter_: Adapter = ADAPTER_CONSTRUCTOR_DICTINARY[self._adapter_name_](
            self
        )

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
        Computes the SLAM's execution time
        """

        self.get_logger().debug("Received map")
        self.map_receive_time_ = datetime.datetime.now()
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
            velocities (np.ndarray): Vehicle state estimation velocities. [v, v, w]
            groundtruth_velocities (np.ndarray): Ground truth vehicle state velocities. [vx, vy, w]
            map (np.ndarray): Map data. [[x,y,color,confidence]]
            groundtruth_map (np.ndarray): Ground truth map data. [[x,y,color,confidence]]
        """
        if map.size == 0 or groundtruth_map.size == 0:
            return
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

        self._vehicle_state_difference_.publish(vehicle_state_error)
        self._map_mean_difference_.publish(mean_difference)
        self._map_mean_squared_difference_.publish(mean_squared_difference)
        self._map_root_mean_squared_difference_.publish(root_mean_squared_difference)


        self._se_map_sum_error += get_average_difference(
            cone_positions, groundtruth_cone_positions)
        self._se_map_squared_sum_error += get_mean_squared_difference(
            cone_positions, groundtruth_cone_positions)
        self._se_map_mean_root_squared_sum_error += get_mean_squared_difference(
            cone_positions, groundtruth_cone_positions) ** (1/2)
        self._se_count += 1

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
        mean_squared_vehicle_state_error = [0.0] * 6
        mean_root_squared_vehicle_state_error = [0.0] * 6

        # Update pose and velocity errors
        for i in range(6):
            self._sum_vehicle_state_error.data[i] += vehicle_state_error.data[i]
            self._sum_squared_vehicle_state_error += vehicle_state_error.data[i] ** 2
            mean_vehicle_state_error.data[i] = self._sum_vehicle_state_error.data[i] / self._se_count
            mean_squared_vehicle_state_error.data[i] = self._sum_squared_vehicle_state_error / self._se_count
            mean_root_squared_vehicle_state_error.data[i] = sqrt(self._sum_squared_vehicle_state_error) / self._se_count
        
        # publish pose and velocity errors
        self._state_estimation_mean_mean_state_error.publish(vehicle_state_error)
        self._state_estimation_mean_mean_state_squared_error.publish(mean_squared_vehicle_state_error)
        self._state_estimation_mean_mean_state_root_squared_error.publish(mean_root_squared_vehicle_state_error)

        mean_mean_error = Float32()
        mean_mean_error.data = self._se_sum_error / self._se_count
        self._state_estimation_mean_mean_error.publish(mean_mean_error)

        mean_mean_squared_error = Float32()
        mean_mean_squared_error.data = self._se_squared_sum_error / self._se_count
        self._state_estimation_mean_mean_squared_error.publish(mean_mean_squared_error)

        mean_mean_root_squared_error = Float32()
        mean_mean_root_squared_error.data = self._se_mean_root_squared_sum_error / self._se_count
        self._state_estimation_mean_mean_root_squared_error.publish(mean_mean_root_squared_error)

    def compute_and_publish_perception(
        self, perception_output: np.ndarray, perception_ground_truth: np.ndarray
    ) -> None:
        """!
        Computes perception metrics and publishes them.

        Args:
            perception_output (np.ndarray): Perceived cones.
            perception_ground_truth (np.ndarray): Ground truth cones.
        """
        cone_positions = np.append(perception_output[:2], 0)
        groundtruth_cone_positions = perception_ground_truth

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

        self._perception_sum_error += get_average_difference(
            cone_positions, groundtruth_cone_positions)
        self._perception_squared_sum_error += get_mean_squared_difference(
            cone_positions, groundtruth_cone_positions)
        self._perception_root_squared_sum_error += get_mean_squared_difference(
            cone_positions, groundtruth_cone_positions) ** (1/2)
        self._perception_count += 1

        mean_mean_error = Float32()
        mean_mean_error.data = self._perception_sum_error / self._perception_count
        self._perception_mean_mean_error.publish(mean_mean_error)

        mean_mean_squared_error = Float32()
        mean_mean_squared_error.data = self._perception_squared_sum_error / self._perception_count
        self._perception_mean_mean_squared_error.publish(mean_mean_squared_error)

        mean_mean_root_squared_error = Float32()
        mean_mean_root_squared_error.data = self._perception_root_squared_sum_error / self._perception_count
        self._perception_mean_mean_root_squared_error.publish(mean_mean_root_squared_error)

        self.get_logger().debug(
            "Computed perception metrics:\n \
                               Mean difference: {}\n \
                               Inter cones distance: {}\n \
                               Mean squared difference: {}\n \
                               Root mean squared difference: {}".format(
                mean_difference,
                inter_cones_distance,
                mean_squared_error,
                root_mean_squared_difference,
            )
        )

        # Publishes computed perception metrics
        self._perception_mean_difference_.publish(mean_difference)
        self._perception_inter_cones_distance_.publish(inter_cones_distance)
        self._perception_mean_squared_difference_.publish(mean_squared_error)
        self._perception_root_mean_squared_difference_.publish(
            root_mean_squared_difference
        )

    def compute_and_publish_planning(self, msg: PathPointArray):
        """!
        Computes planning metrics and publishes them.

        Args:
            msg (PathPointArray): Path points array message.
        """

        self.get_logger().debug("Received planning")
        actual_path = format_path_point_array_msg(msg.pathpoint_array)
        expected_path = format_path_point_array_msg(self.planning_mock)

        self._planning_receive_time_ = datetime.datetime.now()
        time_difference = float(
            (self._planning_receive_time_ - self.map_receive_time_).microseconds / 1000
        )
        execution_time = Float32()
        execution_time.data = time_difference

        self._planning_execution_time_.publish(execution_time)

        if len(actual_path) == 0 or len(expected_path) == 0:
            self.get_logger().debug("Path info missing")
            return

        mean_difference = Float32()
        mean_difference.data = get_average_difference(actual_path, expected_path)

        mean_squared_error = Float32()
        mean_squared_error.data = get_mean_squared_difference(
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
                mean_squared_error,
                root_mean_squared_difference,
            )
        )

        self._planning_mean_difference_.publish(mean_difference)
        self._planning_mean_squared_difference_.publish(mean_squared_error)
        self._planning_root_mean_squared_difference_.publish(
            root_mean_squared_difference
        )

        self._planning_sum_error += get_average_difference(
            actual_path, expected_path)
        self._planning_squared_sum_error += get_mean_squared_difference(
            actual_path, expected_path)
        self._planning_mean_root_squared_sum_error += get_mean_squared_difference(
            actual_path, expected_path) ** (1/2)
        self._planning_count += 1

        mean_mean_error = Float32()
        mean_mean_error.data = self._planning_sum_error / self._planning_count
        self._planning_mean_mean_error.publish(mean_mean_error)

        mean_mean_squared_error = Float32()
        mean_mean_squared_error.data = self._planning_squared_sum_error / self._planning_count
        self._planning_mean_mean_squared_error.publish(mean_mean_squared_error)

        mean_mean_root_squared_error = Float32()
        mean_mean_root_squared_error.data = self._planning_mean_root_squared_sum_error / self._planning_count
        self._planning_mean_mean_root_squared_error.publish(mean_mean_root_squared_error)

    def planning_gt_callback(self, msg: PathPointArray):
        """!
        Stores the path planning ground truth from mocker node.

        Args:
            msg (PathPointArray): Path points array message.
        """
        self.get_logger().debug("Received GT planning")
        self.planning_mock = msg.pathpoint_array

    def compute_and_publish_control(self, msg: EvaluatorControlData):
        """!
        Computes control metrics and publishes them.
        Args:
            vehicle_state (VehicleState): Vehicle state message.
            closest_point (PathPoint): Closest point message.
        """

        self.get_logger().debug("Received control")
        self._control_receive_time_ = datetime.datetime.now()

        pose_treated, velocities_treated = format_vehicle_state_msg(msg.vehicle_state)
        lookahead_point = format_point2d_msg(msg.lookahead_point)
        lookahead_velocity = msg.lookahead_velocity

        time_difference = float(
            (self._control_receive_time_ - self._planning_receive_time_).microseconds
            / 1000
        )
        execution_time = Float32()
        execution_time.data = time_difference

        self._control_execution_time_.publish(execution_time)

        pose_position = pose_treated[:2]
        closest_point = format_point2d_msg(msg.closest_point)

        difference = Float32()
        difference.data = compute_distance(closest_point, pose_position)


        self._control_sum_error += compute_distance(closest_point, pose_position)
        self._control_squared_sum_error += compute_distance(closest_point, pose_position) ** 2
        self._control_velocity_sum_error += abs(velocities_treated - lookahead_velocity)
        self._control_velocity_squared_sum_error += abs(velocities_treated - lookahead_velocity) ** 2
        self._control_count += 1

        velocity_difference = Float32()
        velocity_difference.data = abs(velocities_treated - lookahead_velocity)

        velocity_mean_difference = Float32()
        velocity_mean_difference.data = self._control_velocity_sum_error / self._control_count

        velocity_mean_squared_difference = Float32()
        velocity_mean_squared_difference.data = self._control_velocity_squared_sum_error / self._control_count

        velocity_root_mean_squared = Float32()
        velocity_root_mean_squared.data = sqrt(self._control_velocity_squared_sum_error / self._control_count)

        mean_difference = Float32()
        mean_difference.data = self._control_sum_error / self._control_count

        mean_squared_difference = Float32()
        mean_squared_difference.data = self._control_squared_sum_error / self._control_count

        root_mean_squared_difference = Float32()
        root_mean_squared_difference.data = sqrt(self._control_squared_sum_error / self._control_count)

        self.get_logger().debug(
            "Computed control metrics:\n \
                                Difference: {}\n \
                                Mean difference: {}\n \
                                Mean squared difference: {}\n \
                                Root mean squared difference: {}".format(
                difference,
                mean_difference,
                mean_squared_difference,
                root_mean_squared_difference,
            )
        )

        # Publish control metrics
        self._control_difference_.publish(difference)
        self._control_difference_mean_.publish(mean_difference)
        self._control_mean_squared_difference_.publish(mean_squared_difference)
        self._control_root_mean_squared_difference_.publish(root_mean_squared_difference)

        self._control_velocity_difference_.publish(velocity_difference)
        self._control_velocity_difference_mean_.publish(velocity_mean_difference)
        self._control_mean_squared_difference_.publish(velocity_mean_squared_difference)
        self._control_velocity_mean_squared_difference_.publish(velocity_root_mean_squared)


def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
