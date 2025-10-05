#!/usr/bin/env python3

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from custom_interfaces.msg import (
    PathPointArray,
    Velocities,
    ConeArray,
    Pose,
    ControlParameters,
    PerceptionParameters,
    PlanningParameters,
    SlamParameters,
    VelocityEstimationParameters,
)

CONTROL_PATH = "config/control/vehicle.yaml"
PERCEPTION_PATH = "config/perception/vehicle.yaml"
PLANNING_PATH = "config/planning/vehicle.yaml"
SLAM_PATH = "config/slam/vehicle.yaml"
VELOCITY_ESTIMATION_PATH = "config/velocity_estimation/vehicle.yaml"


class DataInfrastructureNode(Node):
    def __init__(self):
        super().__init__("data_infrastructure_node")
        self.shutdown = False
        self.get_logger().info("Subscribing the publishers.")
        # Control publishers
        self.create_subscription(
            Marker, "/control/visualization/closest_point", self.common_callback, 10
        )
        self.create_subscription(
            Marker, "/control/visualization/lookahead_point", self.common_callback, 10
        )
        # Planning publishers
        self.create_subscription(
            PathPointArray, "/path_planning/path", self.common_callback, 10
        )
        self.create_subscription(
            Float64, "/path_planning/execution_time", self.common_callback, 10
        )
        self.create_subscription(
            Marker, "/path_planning/smoothed_path", self.common_callback, 10
        )
        self.create_subscription(
            MarkerArray, "/path_planning/triangulations", self.common_callback, 10
        )
        # Slam publishers
        self.create_subscription(
            ConeArray, "/state_estimation/map", self.common_callback, 10
        )
        self.create_subscription(
            Pose, "/state_estimation/vehicle_pose", self.common_callback, 10
        )
        self.create_subscription(
            MarkerArray, "/state_estimation/visualization_map", self.common_callback, 10
        )
        self.create_subscription(
            MarkerArray,
            "/state_estimation/visualization_map_perception",
            self.common_callback,
            10,
        )
        self.create_subscription(
            Marker, "/state_estimation/visualization/position", self.common_callback, 10
        )
        self.create_subscription(
            Float64MultiArray,
            "/state_estimation/slam_execution_time",
            self.common_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/state_estimation/slam_covariance",
            self.common_callback,
            10,
        )
        self.create_subscription(
            Float64, "/state_estimation/lap_counter", self.common_callback, 10
        )
        # Velocity Estimation publishers
        self.create_subscription(
            Velocities, "/state_estimation/velocities", self.common_callback, 10
        )

        # Publishers
        self.control_par_pub = self.create_publisher(
            ControlParameters, "data_infrastructure/control_parameters", 10
        )
        self.perception_par_pub = self.create_publisher(
            PerceptionParameters, "data_infrastructure/perception_parameters", 10
        )
        self.planning_par_pub = self.create_publisher(
            PlanningParameters, "data_infrastructure/planning_parameters", 10
        )
        self.slam_par_pub = self.create_publisher(
            SlamParameters, "data_infrastructure/slam_parameters", 10
        )
        self.velocity_par_pub = self.create_publisher(
            VelocityEstimationParameters,
            "data_infrastructure/velocity_estimation_parameters",
            10,
        )

    def load_yaml(self, path, root_key):
        try:
            with open(path, "r") as file:
                data = yaml.safe_load(file)
                return data.get(root_key, {})
        except Exception as e:
            self.get_logger().error(f"Failed to load {path}: {e}")
            return {}

    def control_parameters(self):
        control_msg = ControlParameters()
        parameters = self.load_yaml(CONTROL_PATH, "control")
        control_msg.lookahead_gain = float(parameters.get("lookahead_gain", 0.0))
        control_msg.pid_kp = float(parameters.get("pid_kp", 0.0))
        control_msg.pid_ki = float(parameters.get("pid_ki", 0.0))
        control_msg.pid_kd = float(parameters.get("pid_kd", 0.0))
        control_msg.pid_tau = float(parameters.get("pid_tau", 0.0))
        control_msg.pid_t = float(parameters.get("pid_t", 0.0))
        control_msg.pid_lim_min = float(parameters.get("pid_lim_min", 0.0))
        control_msg.pid_lim_max = float(parameters.get("pid_lim_max", 0.0))
        control_msg.pid_anti_windup = float(parameters.get("pid_anti_windup", 0.0))
        control_msg.lpf_alpha = float(parameters.get("lpf_alpha", 0.0))
        control_msg.lpf_initial_value = float(parameters.get("lpf_initial_value", 0.0))
        self.control_par_pub.publish(control_msg)

    def perception_parameters(self):
        perception_msg = PerceptionParameters()
        parameters = self.load_yaml(PERCEPTION_PATH, "perception")

        # TRACKDRIVE/AUTOCROSS
        perception_msg.horizontal_resolution = float(
            parameters.get("horizontal_resolution", 0.0)
        )
        perception_msg.vertical_resolution = float(
            parameters.get("vertical_resolution", 0.0)
        )
        perception_msg.default_mission = parameters.get("default_mission", "")

        # FOV
        perception_msg.lidar_height = float(parameters.get("lidar_height", 0.0))
        perception_msg.lidar_rotation = float(parameters.get("lidar_rotation", 0.0))
        perception_msg.lidar_pitch = float(parameters.get("lidar_pitch", 0.0))
        perception_msg.min_range = float(parameters.get("min_range", 0.0))
        perception_msg.max_height = float(parameters.get("max_height", 0.0))

        # TRACKDRIVE/AUTOCROSS
        perception_msg.max_range = float(parameters.get("max_range", 0.0))
        perception_msg.fov_trim_angle = float(parameters.get("fov_trim_angle", 0.0))

        # ACCELERATION
        perception_msg.acc_max_range = float(parameters.get("acc_max_range", 0.0))
        perception_msg.acc_fov_trim_angle = float(
            parameters.get("acc_fov_trim_angle", 0.0)
        )
        perception_msg.acc_max_y = float(parameters.get("acc_max_y", 0.0))

        # SKIDPAD
        perception_msg.skid_max_range = float(parameters.get("skid_max_range", 0.0))
        perception_msg.skid_min_distance_to_cone = float(
            parameters.get("skid_min_distance_to_cone", 0.0)
        )

        # Ground removal
        perception_msg.ground_removal = parameters.get("ground_removal", "")
        perception_msg.ransac_epsilon = float(parameters.get("ransac_epsilon", 0.0))
        perception_msg.ransac_iterations = int(parameters.get("ransac_iterations", 0))

        # TRACKDRIVE/AUTOCROSS GROUND REMOVAL
        perception_msg.n_angular_grids = int(parameters.get("n_angular_grids", 0))
        perception_msg.radius_resolution = float(
            parameters.get("radius_resolution", 0.0)
        )

        # ACCELERATION GROUND REMOVAL
        perception_msg.acc_n_angular_grids = int(
            parameters.get("acc_n_angular_grids", 0)
        )
        perception_msg.acc_radius_resolution = float(
            parameters.get("acc_radius_resolution", 0.0)
        )

        # SKIDPAD GROUND REMOVAL
        perception_msg.skid_n_angular_grids = int(
            parameters.get("skid_n_angular_grids", 0)
        )
        perception_msg.skid_radius_resolution = float(
            parameters.get("skid_radius_resolution", 0.0)
        )

        # Clustering
        perception_msg.clustering_n_neighbours = int(
            parameters.get("clustering_n_neighbours", 0)
        )
        perception_msg.clustering_epsilon = float(
            parameters.get("clustering_epsilon", 0.0)
        )

        # Evaluator
        perception_msg.min_height = float(parameters.get("min_height", 0.0))
        perception_msg.large_max_height = float(parameters.get("large_max_height", 0.0))
        perception_msg.small_max_height = float(parameters.get("small_max_height", 0.0))
        perception_msg.height_cap = float(parameters.get("height_cap", 0.0))
        perception_msg.min_xoy = float(parameters.get("min_xoy", 0.0))
        perception_msg.max_xoy = float(parameters.get("max_xoy", 0.0))
        perception_msg.min_z = float(parameters.get("min_z", 0.0))
        perception_msg.max_z = float(parameters.get("max_z", 0.0))
        perception_msg.min_z_score_x = float(parameters.get("min_z_score_x", 0.0))
        perception_msg.max_z_score_x = float(parameters.get("max_z_score_x", 0.0))
        perception_msg.min_z_score_y = float(parameters.get("min_z_score_y", 0.0))
        perception_msg.max_z_score_y = float(parameters.get("max_z_score_y", 0.0))
        perception_msg.out_distance_cap = float(parameters.get("out_distance_cap", 0.0))
        perception_msg.min_distance_x = float(parameters.get("min_distance_x", 0.0))
        perception_msg.min_distance_y = float(parameters.get("min_distance_y", 0.0))
        perception_msg.min_distance_z = float(parameters.get("min_distance_z", 0.0))
        perception_msg.min_n_points = int(parameters.get("min_n_points", 0))
        perception_msg.min_confidence = float(parameters.get("min_confidence", 0.0))
        perception_msg.height_out_weight = float(
            parameters.get("height_out_weight", 0.0)
        )
        perception_msg.height_in_weight = float(parameters.get("height_in_weight", 0.0))
        perception_msg.cylinder_radius_weight = float(
            parameters.get("cylinder_radius_weight", 0.0)
        )
        perception_msg.cylinder_height_weight = float(
            parameters.get("cylinder_height_weight", 0.0)
        )
        perception_msg.cylinder_npoints_weight = float(
            parameters.get("cylinder_npoints_weight", 0.0)
        )
        perception_msg.npoints_weight = float(parameters.get("npoints_weight", 0.0))
        perception_msg.displacement_x_weight = float(
            parameters.get("displacement_x_weight", 0.0)
        )
        perception_msg.displacement_y_weight = float(
            parameters.get("displacement_y_weight", 0.0)
        )
        perception_msg.displacement_z_weight = float(
            parameters.get("displacement_z_weight", 0.0)
        )
        perception_msg.deviation_xoy_weight = float(
            parameters.get("deviation_xoy_weight", 0.0)
        )
        perception_msg.deviation_z_weight = float(
            parameters.get("deviation_z_weight", 0.0)
        )
        self.perception_par_pub.publish(perception_msg)

    def planning_parameters(self):
        planning_msg = PlanningParameters()
        parameters = self.load_yaml(PLANNING_PATH, "planning")
        planning_msg.nc_angle_gain = float(parameters.get("nc_angle_gain", 0.0))
        planning_msg.nc_distance_gain = float(parameters.get("nc_distance_gain", 0.0))
        planning_msg.nc_angle_exponent = float(parameters.get("nc_angle_exponent", 0.0))
        planning_msg.nc_distance_exponent = float(
            parameters.get("nc_distance_exponent", 0.0)
        )
        planning_msg.nc_max_cost = float(parameters.get("nc_max_cost", 0.0))
        planning_msg.nc_search_depth = int(parameters.get("nc_search_depth", 0))
        planning_msg.nc_max_points = int(parameters.get("nc_max_points", 0))
        planning_msg.outliers_spline_order = int(
            parameters.get("outliers_spline_order", 0)
        )
        planning_msg.outliers_spline_coeffs_ratio = float(
            parameters.get("outliers_spline_coeffs_ratio", 0.0)
        )
        planning_msg.outliers_spline_precision = int(
            parameters.get("outliers_spline_precision", 0)
        )
        planning_msg.path_calculation_dist_threshold = float(
            parameters.get("path_calculation_dist_threshold", 0.0)
        )
        planning_msg.smoothing_spline_order = int(
            parameters.get("smoothing_spline_order", 0)
        )
        planning_msg.smoothing_spline_coeffs_ratio = float(
            parameters.get("smoothing_spline_coeffs_ratio", 0.0)
        )
        planning_msg.smoothing_spline_precision = int(
            parameters.get("smoothing_spline_precision", 0)
        )
        planning_msg.publishing_visualization_msg = bool(
            parameters.get("publishing_visualization_msg", False)
        )
        planning_msg.desired_velocity = float(parameters.get("desired_velocity", 0.0))
        planning_msg.use_outlier_removal = bool(
            parameters.get("use_outlier_removal", False)
        )
        planning_msg.use_path_smoothing = bool(
            parameters.get("use_path_smoothing", False)
        )
        planning_msg.minimum_velocity = float(parameters.get("minimum_velocity", 0.0))
        planning_msg.braking_acceleration = float(
            parameters.get("braking_acceleration", 0.0)
        )
        planning_msg.normal_acceleration = float(
            parameters.get("normal_acceleration", 0.0)
        )
        planning_msg.use_velocity_planning = bool(
            parameters.get("use_velocity_planning", False)
        )
        self.planning_par_pub.publish(planning_msg)

    def slam_parameters(self):
        slam_msg = SlamParameters()
        parameters = self.load_yaml(SLAM_PATH, "slam")
        slam_msg.motion_model_name = parameters.get("motion_model_name", "")
        slam_msg.landmark_filter_name = parameters.get("landmark_filter_name", "")
        slam_msg.data_association_model_name = parameters.get(
            "data_association_model_name", ""
        )
        slam_msg.data_association_limit_distance = float(
            parameters.get("data_association_limit_distance", 0.0)
        )
        slam_msg.data_association_gate = float(
            parameters.get("data_association_gate", 0.0)
        )
        slam_msg.new_landmark_confidence_gate = float(
            parameters.get("new_landmark_confidence_gate", 0.0)
        )
        slam_msg.observation_x_noise = float(parameters.get("observation_x_noise", 0.0))
        slam_msg.observation_y_noise = float(parameters.get("observation_y_noise", 0.0))
        slam_msg.velocity_x_noise = float(parameters.get("velocity_x_noise", 0.0))
        slam_msg.velocity_y_noise = float(parameters.get("velocity_y_noise", 0.0))
        slam_msg.angular_velocity_noise = float(
            parameters.get("angular_velocity_noise", 0.0)
        )
        slam_msg.pose_x_initial_noise = float(
            parameters.get("pose_x_initial_noise", 0.0)
        )
        slam_msg.pose_y_initial_noise = float(
            parameters.get("pose_y_initial_noise", 0.0)
        )
        slam_msg.pose_theta_initial_noise = float(
            parameters.get("pose_theta_initial_noise", 0.0)
        )
        slam_msg.slam_solver_name = parameters.get("slam_solver_name", "")
        slam_msg.minimum_observation_count = int(
            parameters.get("minimum_observation_count", 0)
        )
        slam_msg.minimum_frequency_of_detections = int(
            parameters.get("minimum_frequency_of_detections", 0)
        )
        slam_msg.slam_min_pose_difference = float(
            parameters.get("slam_min_pose_difference", 0.0)
        )
        slam_msg.slam_optimization_mode = parameters.get("slam_optimization_mode", "")
        slam_msg.slam_optimization_type = parameters.get("slam_optimization_type", "")
        slam_msg.preloaded_map_noise = float(parameters.get("preloaded_map_noise", 0.0))
        slam_msg.slam_optimization_period = float(
            parameters.get("slam_optimization_period", 0.0)
        )
        slam_msg.slam_isam2_relinearize_threshold = float(
            parameters.get("slam_isam2_relinearize_threshold", 0.0)
        )
        slam_msg.slam_isam2_relinearize_skip = int(
            parameters.get("slam_isam2_relinearize_skip", 0)
        )
        slam_msg.slam_isam2_factorization = parameters.get(
            "slam_isam2_factorization", ""
        )
        slam_msg.sliding_window_size = int(parameters.get("sliding_window_size", 0))
        self.slam_par_pub.publish(slam_msg)

    def velocity_parameters(self):
        velocity_msg = VelocityEstimationParameters()
        parameters = self.load_yaml(VELOCITY_ESTIMATION_PATH, "velocity_estimation")
        velocity_msg.estimation_method = parameters.get("estimation_method", "")
        velocity_msg.ve_observation_model_name = parameters.get(
            "ve_observation_model_name", ""
        )
        velocity_msg.process_model_name = parameters.get("process_model_name", "")
        velocity_msg.wheel_speed_noise = float(parameters.get("wheel_speed_noise", 0.0))
        velocity_msg.motor_rpm_noise = float(parameters.get("motor_rpm_noise", 0.0))
        velocity_msg.steering_angle_noise = float(
            parameters.get("steering_angle_noise", 0.0)
        )
        velocity_msg.imu_acceleration_noise = float(
            parameters.get("imu_acceleration_noise", 0.0)
        )
        velocity_msg.imu_rotational_noise = float(
            parameters.get("imu_rotational_noise", 0.0)
        )
        velocity_msg.angular_velocity_process_noise = float(
            parameters.get("angular_velocity_process_noise", 0.0)
        )
        self.velocity_par_pub.publish(velocity_msg)

    def common_callback(self, msg):
        self.get_logger().info("Publishing the parameters.")
        self.control_parameters()
        self.perception_parameters()
        self.planning_parameters()
        self.slam_parameters()
        self.velocity_parameters()
        self.shutdown = True


def main(args=None):
    rclpy.init(args=args)
    node = DataInfrastructureNode()
    while not node.shutdown:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
