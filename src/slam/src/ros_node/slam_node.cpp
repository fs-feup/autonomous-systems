#include "ros_node/slam_node.hpp"

#include <fstream>

#include "common_lib/communication/marker.hpp"
#include "common_lib/maths/transformations.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/position.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "motion_lib/bicycle_model.hpp"
#include "motion_lib/car_parameters.hpp"
#include "visualization_msgs/msg/marker.hpp"

/*---------------------- Constructor --------------------*/

// SLAMNode::SLAMNode() : Node("slam") {
//   // Parameters initialization
//   _use_simulated_perception_ = this->declare_parameter("use_simulated_perception", false);
//   _adapter_name_ = this->declare_parameter("adapter", "eufs");
//   std::string motion_model_name = this->declare_parameter("motion_model",
//   "normal_velocity_model"); std::string data_assocation_model_name =
//       this->declare_parameter("data_assocation_model", "max_likelihood");
//   float wss_noise = 0.0f;
//   float imu_noise = 0.0f;  // Declare the 'imu_noise' variable
//   if (data_assocation_model_name == "max_likelihood") {
//     wss_noise = static_cast<float>(this->declare_parameter("wss_noise", 0.3f));
//     imu_noise = static_cast<float>(this->declare_parameter("imu_noise", 0.0064f));
//   }
//   float data_association_limit_distance =
//       static_cast<float>(this->declare_parameter("data_association_limit_distance", 71.0f));
//   float observation_noise = static_cast<float>(this->declare_parameter("observation_noise",
//   0.03f));

//   // Initialize the models
//   // std::shared_ptr<MotionModel> motion_model_wss = motion_model_constructors.at(
//   // "normal_velocity_model")(MotionModel::create_process_noise_covariance_matrix(wss_noise));
//   // std::shared_ptr<MotionModel> motion_model_imu =
//   // motion_model_constructors.at(motion_model_name)(
//   //     MotionModel::create_process_noise_covariance_matrix(imu_noise));
//   // std::shared_ptr<ObservationModel> observation_model = std::make_shared<ObservationModel>(
//   //     ObservationModel::create_observation_noise_covariance_matrix(observation_noise));
//   // std::shared_ptr<DataAssociationModel> data_association_model =
//   //     data_association_model_constructors.at(data_assocation_model_name)(
//   //         data_association_limit_distance);

//   // Initialize SLAM solver object
//   // _ekf_ = std::make_shared<ExtendedKalmanFilter>(observation_model, data_association_model);
//   // _ekf_->add_motion_model("wheel_speed_sensor", motion_model_wss);
//   // _ekf_->add_motion_model("imu", motion_model_imu);

//   // Initialize data structures
//   _perception_map_ = std::make_shared<std::vector<common_lib::structures::Cone>>();
//   _vehicle_state_velocities_ = std::make_shared<common_lib::structures::Velocities>();
//   _track_map_ = std::make_shared<std::vector<common_lib::structures::Cone>>();
//   _vehicle_pose_ = std::make_shared<common_lib::structures::Pose>();

//   // Subscriptions
//   if (!_use_simulated_perception_) {
//     this->_perception_subscription_ =
//     this->create_subscription<custom_interfaces::msg::ConeArray>(
//         "/perception/cones", 1,
//         std::bind(&SLAMNode::_perception_subscription_callback, this, std::placeholders::_1));
//   }
//   if (!_use_simulated_velocities_) {
//     this->_velocities_subscription_ =
//     this->create_subscription<custom_interfaces::msg::Velocities>(
//         "/vehicle/velocities", 1,
//         std::bind(&SLAMNode::_velocities_subscription_callback, this, std::placeholders::_1));
//   }

//   // Publishers
//   this->_map_publisher_ =
//       this->create_publisher<custom_interfaces::msg::ConeArray>("/state_estimation/map", 10);
//   this->_visualization_map_publisher_ =
//       this->create_publisher<visualization_msgs::msg::MarkerArray>(
//           "/state_estimation/visualization_map", 10);
//   this->_position_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
//       "/state_estimation/visualization/position", 10);
//   this->_correction_execution_time_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
//       "/state_estimation/execution_time/correction_step", 10);
//   this->_prediction_execution_time_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
//       "/state_estimation/execution_time/prediction_step", 10);
// }

SLAMNode::SLAMNode(const SLAMParameters &params) : Node("slam") {
  // Initialize the models

  // Initialize SLAM solver object

  // Parameters initialization
  std::string motion_model_name = params.motion_model_name_;

  _perception_map_ = std::make_shared<std::vector<common_lib::structures::Cone>>();
  _vehicle_state_velocities_ = std::make_shared<common_lib::structures::Velocities>();
  _track_map_ = std::make_shared<std::vector<common_lib::structures::Cone>>();
  _vehicle_pose_ = std::make_shared<common_lib::structures::Pose>();

  // Subscriptions
  if (!params.use_simulated_perception_) {
    this->_perception_subscription_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "/perception/cones", 1,
        std::bind(&SLAMNode::_perception_subscription_callback, this, std::placeholders::_1));
  }
  if (!_use_simulated_velocities_) {
    this->_velocities_subscription_ = this->create_subscription<custom_interfaces::msg::Velocities>(
        "/vehicle/velocities", 1,
        std::bind(&SLAMNode::_velocities_subscription_callback, this, std::placeholders::_1));
  }

  // Publishers
  this->_map_publisher_ =
      this->create_publisher<custom_interfaces::msg::ConeArray>("/state_estimation/map", 10);
  this->_visualization_map_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/state_estimation/visualization_map", 10);
  this->_position_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/state_estimation/visualization/position", 10);
  this->_correction_execution_time_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/state_estimation/execution_time/correction_step", 10);
  this->_prediction_execution_time_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/state_estimation/execution_time/prediction_step", 10);
}

/*---------------------- Subscriptions --------------------*/

void SLAMNode::_perception_subscription_callback(const custom_interfaces::msg::ConeArray &msg) {
  // auto const &cone_array = msg.cone_array;
  // if (this->_perception_map_ == nullptr) {
  //   RCLCPP_WARN(this->get_logger(), "SUB - Perception map is null");
  //   return;
  // }

  // if (!this->_go_) {
  //   return;
  // }

  // rclcpp::Time start_time = this->get_clock()->now();

  // this->_perception_map_->clear();

  // for (auto &cone : cone_array) {
  //   this->_perception_map_->push_back(common_lib::structures::Cone(cone.position.x,
  //   cone.position.y,
  //                                                                  cone.color, cone.confidence));
  // }

  // if (this->_ekf_ == nullptr) {
  //   RCLCPP_WARN(this->get_logger(), "ATTR - EKF object is null");
  //   return;
  // }
  // // RCLCPP_DEBUG(this->get_logger(), "CORRECTION STEP END\n---------------------------\n");
  // this->_ekf_->correction_step(*(this->_perception_map_));
  // this->_ekf_->update(this->_vehicle_state_, this->_track_map_);

  // rclcpp::Time end_time = this->get_clock()->now();

  // // Execution Time calculation
  // std_msgs::msg::Float64 correction_execution_time;
  // correction_execution_time.data = (end_time - start_time).seconds() * 1000.0;
  // this->_correction_execution_time_publisher_->publish(correction_execution_time);
  // // this->_publish_vehicle_state();
  // this->_publish_map();
}

void SLAMNode::_velocities_subscription_callback(const custom_interfaces::msg::Velocities &msg) {
  // TODO(marhcouto): Implement this function
}

/*---------------------- Publications --------------------*/

void SLAMNode::_publish_vehicle_pose() {
  // if (this->_vehicle_state_ == nullptr) {
  //   RCLCPP_WARN(this->get_logger(), "PUB - Vehicle state object is null");
  //   return;
  // }

  // auto message = custom_interfaces::msg::VehicleState();
  // message.position.x = this->_vehicle_state_->pose.position.x;
  // message.position.y = this->_vehicle_state_->pose.position.y;
  // message.theta = this->_vehicle_state_->pose.orientation;
  // double velocity_x = this->_vehicle_state_->velocity_x;
  // double velocity_y = this->_vehicle_state_->velocity_y;
  // message.linear_velocity = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
  // message.angular_velocity = this->_vehicle_state_->rotational_velocity;
  // message.header.stamp = this->get_clock()->now();

  // RCLCPP_DEBUG(this->get_logger(), "PUB - Pose: (%f, %f, %f); Velocities: (%f, %f)",
  //              message.position.x, message.position.y, message.theta, message.linear_velocity,
  //              message.angular_velocity);
  // this->_vehicle_state_publisher_->publish(message);
}

void SLAMNode::_publish_map() {
  // auto cone_array_msg = custom_interfaces::msg::ConeArray();
  // auto marker_array_msg = visualization_msgs::msg::MarkerArray();
  // RCLCPP_DEBUG(this->get_logger(), "PUB - cone map:");
  // RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
  // for (common_lib::structures::Cone const &cone : *this->_track_map_) {
  //   auto cone_message = custom_interfaces::msg::Cone();
  //   cone_message.position.x = cone.position.x;
  //   cone_message.position.y = cone.position.y;
  //   cone_message.color = common_lib::competition_logic::get_color_string(cone.color);
  //   cone_array_msg.cone_array.push_back(cone_message);
  //   RCLCPP_DEBUG(this->get_logger(), "(%f\t%f)\t%s", cone_message.position.x,
  //                cone_message.position.y, cone_message.color.c_str());
  // }
  // RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
  // cone_array_msg.header.stamp = this->get_clock()->now();
  // marker_array_msg = common_lib::communication::marker_array_from_structure_array(
  //     *this->_track_map_, "map_cones", _adapter_name_ == "eufs" ? "base_footprint" : "map");
  // this->_map_publisher_->publish(cone_array_msg);
  // this->_visualization_map_publisher_->publish(marker_array_msg);
}
