from evaluator.adapter import Adapter
from eufs_msgs.msg import ConeArrayWithCovariance, CarState
from nav_msgs.msg import Odometry
from custom_interfaces.msg import ConeArray, VehicleState, PathPointArray
import rclpy
import datetime
import message_filters
import numpy as np
from evaluator.formats import (
    format_cone_array_msg,
    format_vehicle_state_msg,
    format_eufs_cone_array_with_covariance_msg,
    format_nav_odometry_msg,
    format_car_state_msg,
    format_path_point_array_msg,
    get_blue_and_yellow_cones_after_msg_treatment,
)


class EufsAdapter(Adapter):
    """!
    Adapter class for subscribing to EUFS topics
    """

    def __init__(self, node: rclpy.node.Node):
        """!
        Initializes the EUFS Adapter.

        Args:
            node (Node): ROS2 node instance.
        """

        super().__init__(node)
        self.groundtruth_vehicle_state_ = None
        self.groundtruth_map_ = None
        self.simulated_vehicle_state_ = None
        self.node.groundtruth_map_subscription_ = self.node.create_subscription(
            ConeArrayWithCovariance,
            "/ground_truth/track",
            self.groundtruth_map_callback,
            10,
        )

        self.node.groundtruth_state_subscription_ = self.node.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self.groundtruth_vehicle_state_callback,
            10,
        )

        self.node.simulated_state_subscription = self.node.create_subscription(
            CarState,
            "/odometry_integration/car_state",
            self.simulated_vehicle_state_callback,
            10,
        )

        self._se_time_sync_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.vehicle_state_subscription_,
                self.node.map_subscription_,
            ],
            10,
            0.5,
        )

        self._se_time_sync_.registerCallback(self.state_estimation_callback)

        self.node.groundtruth_perception_subscription_ = message_filters.Subscriber(
            self.node, ConeArrayWithCovariance, "/ground_truth/cones"
        )
        self._perception_time_sync_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.perception_subscription_,
                self.node.groundtruth_perception_subscription_,
            ],
            10,
            0.1,
        )

        self._perception_time_sync_.registerCallback(self.perception_callback)

        self._planning_time_sync_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.planning_subscription_,
                self.node.planning_gt_subscription_,
            ],
            10,
            0.5,
        )

        self._planning_time_sync_.registerCallback(self.planning_callback)

    def simulated_vehicle_state_callback(self, msg: CarState):
        """!
        Callback function to mark the initial timestamp of the control execution

        Args:
            msg (CarState): Car state coming from EUFS simulator
        """
        self.simulated_vehicle_state_ = msg

    def state_estimation_callback(
        self,
        vehicle_state: VehicleState,
        map: ConeArray,
    ):
        """!
        Callback function to process synchronized messages and compute state estimation metrics.

        Args:
            vehicle_state (VehicleState): Vehicle state estimation message.
            map (ConeArray): Cone array message.
        """
        if (
            self.groundtruth_vehicle_state_ is None
            or self.groundtruth_map_ is None
            or self.simulated_vehicle_state_ is None
        ):
            return
        pose_treated, velocities_treated = format_vehicle_state_msg(vehicle_state)
        map_treated: np.ndarray = format_cone_array_msg(map)
        groundtruth_pose_treated, groundtruth_velocity_treated = (
            format_nav_odometry_msg(self.groundtruth_vehicle_state_)
        )
        groundtruth_map_treated: np.ndarray = (
            format_eufs_cone_array_with_covariance_msg(self.groundtruth_map_)
        )
        self.node.compute_and_publish_state_estimation(
            pose_treated,
            groundtruth_pose_treated,
            velocities_treated,
            groundtruth_velocity_treated,
            map_treated,
            groundtruth_map_treated,
        )

    def perception_callback(
        self, perception_output: ConeArray, ground_truth: ConeArrayWithCovariance
    ):
        """!
        Callback function to process synchronized messages and compute perception metrics.

        Args:
            perception_output (ConeArray): Perception Output.
        """

        perception_treated: np.ndarray = format_cone_array_msg(perception_output)

        groundtruth_perception_treated: np.ndarray = (
            format_eufs_cone_array_with_covariance_msg(ground_truth)
        )

        self.node.compute_and_publish_perception(
            perception_treated, groundtruth_perception_treated
        )

    def planning_callback(
        self,
        planning_output: PathPointArray,
        path_ground_truth: PathPointArray,
    ):
        """!
        Callback function to process synchronized messages and compute planning metrics.

        Args:
            map_ground_truth (ConeArrayWithCovariance): Groundtruth map message.
            planning_output (PathPointArray): Planning output.
            path_ground_truth (PathPointArray): Groundtruth path message.
        """
        if self.groundtruth_map_ is None:
            return
        map_ground_truth_treated: np.ndarray = (
            format_eufs_cone_array_with_covariance_msg(self.groundtruth_map_)
        )
        self.node.get_logger().info(
            "treated gt with size: %d" % len(map_ground_truth_treated)
        )
        blue_cones, yellow_cones = get_blue_and_yellow_cones_after_msg_treatment(
            map_ground_truth_treated
        )
        self.node.get_logger().info(
            "n blue cones: %d; n yellow cones : %d"
            % (len(blue_cones), len(yellow_cones))
        )
        planning_output_treated: np.ndarray = format_path_point_array_msg(
            planning_output
        )
        path_ground_truth_treated: np.ndarray = format_path_point_array_msg(
            path_ground_truth
        )
        self.node.compute_and_publish_planning(
            planning_output_treated, path_ground_truth_treated, blue_cones, yellow_cones
        )

    def groundtruth_map_callback(self, track: ConeArrayWithCovariance):
        """!
        Callback function to process groundtruth map messages.

        Args:
            track (ConeArrayWithCovariance): Groundtruth track data.
        """
        self.node.get_logger().debug(
            "Received groundtruth map with size: %d" % len(track.blue_cones)
        )
        self.groundtruth_map_ = track

    def groundtruth_vehicle_state_callback(self, vehicle_state: Odometry):
        """!
        Callback function to process groundtruth vehicle_state messages.

        Args:
            vehicle_state (Odometry): Groundtruth vehicle_state data.
        """
        self.node.get_logger().debug("Received groundtruth vehicle state")
        self.groundtruth_vehicle_state_ = vehicle_state
