import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ConeArray, VehicleState
from evaluator.adapter import Adapter
from evaluator.metrics import (
    get_mean_squared_difference,
    get_average_difference,
    get_inter_cones_distance,
)
import tf2_ros
import sys
from tf2_ros.transform_listener import TransformListener
from evaluator.adapter_maps import (
    ADAPTER_CONSTRUCTOR_DICTINARY,
    ADAPTER_POINT_CLOUD_TOPIC_DICTINARY,
)
import numpy as np
import rclpy.subscription
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import PointCloud2
import datetime
from math import sqrt
import message_filters


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
            self.declare_parameter("adapter", "pacsim")
            .get_parameter_value()
            .string_value
        )
        self.use_simulated_perception_: bool = (
            self.declare_parameter("use_simulated_perception", False)
            .get_parameter_value()
            .bool_value
        )
        if (self._adapter_name_ == "fsds") and (self.use_simulated_perception_):
            rclpy.get_logger().error(
                "Simulated perception is not supported for FSDS adapter"
            )
            sys.exit(1)

        self.perception_receive_time_: datetime.datetime = datetime.datetime.now()
        self.map_receive_time_: datetime.datetime = datetime.datetime.now()
        self._point_cloud_receive_time_: datetime.datetime = datetime.datetime.now()

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
        Computes perception metrics and publishes them.

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
        vehicle_state_error.data[2] = abs(pose[2] - groundtruth_pose[2])
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

    def compute_and_publish_perception(
        self, perception_output: np.ndarray, perception_ground_truth: np.ndarray
    ) -> None:
        """!
        Computes perception metrics and publishes them.

        Args:
            perception_output (np.ndarray): Perceived cones.
            perception_ground_truth (np.ndarray): Ground truth cones.
        """
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


def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
