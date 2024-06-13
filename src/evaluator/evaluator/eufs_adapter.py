from evaluator.adapter import Adapter
from eufs_msgs.msg import ConeArrayWithCovariance, CarState
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
        self.groundtruth_pose_ = None
        self.groundtruth_map_ = None
        self.node.groundtruth_map_subscription_ = self.node.create_subscription(
            ConeArrayWithCovariance,
            "/ground_truth/track",
            self.groundtruth_map_callback,
            10,
        )
        self.node.groundtruth_pose_subscription_ = self.node.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self.groundtruth_pose_callback,
            10,
        )
        self.node.simulated_perception_subscription_ = self.node.create_subscription(
            ConeArrayWithCovariance,
            "/cones",
            self.simulated_perception_callback,
            10,
        )

        self.node.simulated_planning_subscription = self.node.create_subscription(
            CarState,
            "/odometry_integration/car_state",
            self.set_control_init,
            10,
        )

        self._time_sync_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.vehicle_state_subscription_,
                self.node.map_subscription_,
            ],
            10,
            0.5,
        )

        self._time_sync_.registerCallback(self.state_estimation_callback)
    
    def set_control_init(
        self,
        msg: CarState
    ):
        """!
        Callback function to mark the initial timestamp of the control execution

        Args:
            msg (CarState): Car state coming from EUFS simulator
        """
        if self.node.use_simulated_planning_:
            self.node._planning_receive_time_ = datetime.datetime.now()
        

    def state_estimation_callback(
        self,
        vehicle_state: VehicleState,
        map: ConeArray,
    ):
        """!
        Callback function to process synchronized messages and compute perception metrics.

        Args:
            vehicle_state (VehicleState): Vehicle state estimation message.
            map (ConeArray): Cone array message.
        """
        if self.groundtruth_pose_ is None or self.groundtruth_map_ is None:
            return
        pose_treated, velociies_treated = format_vehicle_state_msg(vehicle_state)
        map_treated: np.ndarray = format_cone_array_msg(map)
        groundtruth_pose_treated: np.ndarray = format_nav_odometry_msg(
            self.groundtruth_pose_
        )
        groundtruth_map_treated: np.ndarray = (
            format_eufs_cone_array_with_covariance_msg(self.groundtruth_map_)
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

    def groundtruth_map_callback(self, track: ConeArrayWithCovariance):
        """!
        Callback function to process groundtruth map messages.

        Args:
            track (ConeArrayWithCovariance): Groundtruth track data.
        """
        self.node.get_logger().debug("Received groundtruth map")
        self.groundtruth_map_ = track

    def groundtruth_pose_callback(self, pose: Odometry):
        """!
        Callback function to process groundtruth pose messages.

        Args:
            pose (Odometry): Groundtruth pose data.
        """
        self.node.get_logger().debug("Received groundtruth pose")
        self.groundtruth_pose_ = pose
        if self.node.use_simulated_se_:
            self.node.map_receive_time_ = datetime.datetime.now()

    def simulated_perception_callback(self, perception: ConeArrayWithCovariance):
        """!
        Callback function to process simulated perception messages.

        Args:
            perception (PerceptionDetections): Simulated perception data.
        """
        if self.node.use_simulated_perception_:
            self.node.perception_receive_time_ = datetime.datetime.now()
