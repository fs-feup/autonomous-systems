from evaluator.adapter import Adapter
from eufs_msgs.msg import ConeArrayWithCovariance
from nav_msgs.msg import Odometry
from custom_interfaces.msg import ConeArray, VehicleState
import rclpy
import datetime
import message_filters
import numpy as np
from evaluator.formats import (
    format_cone_array_msg,
    format_vehicle_state_msg,
    format_eufs_cone_array_with_covariance_msg,
    format_nav_odometry_msg,
)


class EufsAdapter(Adapter):
    """!
    Adapter class for subscribing to PacSim topics
    """

    def __init__(self, node: rclpy.node.Node):
        """!
        Initializes the PacSim Adapter.

        Args:
            node (Node): ROS2 node instance.
        """

        super().__init__(node)
        self.node.groundtruth_map_subscription_ = message_filters.Subscriber(
            self.node,
            ConeArrayWithCovariance,
            "/ground_truth/track",
        )
        self.node.groundtruth_pose_subscription_ = message_filters.Subscriber(
            self.node,
            Odometry,
            "/ground_truth/odom",
        )
        self.node.simulated_perception_subscription_ = self.node.create_subscription(
            ConeArrayWithCovariance,
            "/cones",
            self.simulated_perception_callback,
            10,
        )

        self._time_sync_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.vehicle_state_subscription_,
                self.node.map_subscription_,
                self.node.groundtruth_pose_subscription_,
                self.node.groundtruth_map_subscription_,
            ],
            10,
            0.1,
        )

        self._time_sync_.registerCallback(self.state_estimation_callback)

    def state_estimation_callback(
        self,
        vehicle_state: VehicleState,
        map: ConeArray,
        pose: Odometry,
        track: ConeArrayWithCovariance,
    ):
        """!
        Callback function to process synchronized messages and compute perception metrics.

        Args:
            vehicle_state (VehicleState): Vehicle state estimation message.
            map (ConeArray): Cone array message.
            pose (Odometry): Odometry message from eufs.
            track (ConeArrayWithCovariance): Cone array with covariance message from eufs.
        """
        pose_treated, velociies_treated = format_vehicle_state_msg(vehicle_state)
        map_treated: np.ndarray = format_cone_array_msg(map)
        groundtruth_pose_treated: np.ndarray = format_nav_odometry_msg(pose)
        groundtruth_map_treated: np.ndarray = (
            format_eufs_cone_array_with_covariance_msg(track)
        )
        empty_groundtruth_velocity_treated = np.array([0, 0, 0])
        self.node.compute_and_publish_state_estimation(
            pose_treated,
            groundtruth_pose_treated,
            velociies_treated,
            empty_groundtruth_velocity_treated,
            map_treated,
            groundtruth_map_treated,
        )

    def simulated_perception_callback(self, perception: ConeArrayWithCovariance):
        """!
        Callback function to process simulated perception messages.

        Args:
            perception (PerceptionDetections): Simulated perception data.
        """
        if self.node.use_simulated_perception_:
            self.node.perception_receive_time_ = datetime.datetime.now()
