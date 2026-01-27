#include "adapter_slam/pacsim.hpp"

#include "common_lib/competition_logic/color.hpp"
#include "custom_interfaces/msg/cone.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "ros_node/slam_node.hpp"

PacsimAdapter::PacsimAdapter(const SLAMParameters& params) : SLAMNode(params) {
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = this->_callback_group_;
  if (params.use_simulated_perception_) {
    if (params.slam_solver_name_ == "ekf_slam" && params.slam_optimization_mode_ == "async") {
      this->_parallel_callback_group_ =
          this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      rclcpp::SubscriptionOptions parallel_opts;
      parallel_opts.callback_group = _parallel_callback_group_;

      this->_perception_detections_subscription_ =
          this->create_subscription<pacsim::msg::PerceptionDetections>(
              "/pacsim/perception/lidar/landmarks", 1,
              std::bind(&PacsimAdapter::_pacsim_perception_subscription_callback, this,
                        std::placeholders::_1),
              parallel_opts);
    } else {
      this->_perception_detections_subscription_ =
          this->create_subscription<pacsim::msg::PerceptionDetections>(
              "/pacsim/perception/lidar/landmarks", 1,
              std::bind(&PacsimAdapter::_pacsim_perception_subscription_callback, this,
                        std::placeholders::_1));
    }
  }

  if (params.use_simulated_velocities_) {
    this->_velocities_subscription_ =
        this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/pacsim/velocity", 1,
            std::bind(&PacsimAdapter::_pacsim_velocities_subscription_callback, this,
                      std::placeholders::_1),
            subscription_options);
  }

  this->_imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/pacsim/imu/cog_imu", 1,
      std::bind(&PacsimAdapter::_pacsim_imu_subscription_callback, this, std::placeholders::_1),
      subscription_options);

  this->_finished_client_ = this->create_client<std_srvs::srv::Empty>("/pacsim/finish_signal");
  param_client_ =
      this->create_client<rcl_interfaces::srv::GetParameters>("/pacsim/pacsim_node/get_parameters");
  fetch_discipline();

  this->_go_ = true;  // No go signal needed for pacsim
}

void PacsimAdapter::fetch_discipline() {
  if (!param_client_->wait_for_service(std::chrono::milliseconds(100))) {
    RCLCPP_ERROR(this->get_logger(), "Service /pacsim/pacsim_node/get_parameters not available.");
  } else {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("discipline");

    param_client_->async_send_request(
        request, [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
          auto response = future.get();
          common_lib::competition_logic::Mission mission_result =
              common_lib::competition_logic::Mission::AUTOCROSS;

          if (!response->values.empty() && response->values[0].type == 4) {  // Type 4 = string
            std::string discipline = response->values[0].string_value;
            RCLCPP_INFO(this->get_logger(), "Discipline received: %s", discipline.c_str());

            if (discipline == "skidpad") {
              mission_result = common_lib::competition_logic::Mission::SKIDPAD;
            } else if (discipline == "acceleration") {
              mission_result = common_lib::competition_logic::Mission::ACCELERATION;
            } else if (discipline == "trackdrive") {
              mission_result = common_lib::competition_logic::Mission::TRACKDRIVE;
            } else {
              RCLCPP_ERROR(this->get_logger(), "Unknown discipline received: %s",
                           discipline.c_str());
            }
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to retrieve discipline parameter.");
          }
          this->_mission_ = mission_result;
          this->_slam_solver_->set_mission(mission_result);
        });
  }
}

void PacsimAdapter::finish() {
  this->_finished_client_->async_send_request(
      std::make_shared<std_srvs::srv::Empty::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
        RCLCPP_INFO(this->get_logger(), "Finished signal sent");
      });
}

void PacsimAdapter::_pacsim_perception_subscription_callback(
    const pacsim::msg::PerceptionDetections& msg) {
  custom_interfaces::msg::ConeArray cone_array_msg;
  for (const pacsim::msg::PerceptionDetection& detection : msg.detections) {
    custom_interfaces::msg::Point2d position = custom_interfaces::msg::Point2d();
    position.x = detection.pose.pose.position.x;
    position.y = detection.pose.pose.position.y;
    auto cone_message = custom_interfaces::msg::Cone();
    cone_message.position = position;
    cone_message.confidence = detection.detection_probability;
    cone_message.color = common_lib::competition_logic::get_color_string(
        common_lib::competition_logic::Color::UNKNOWN);
    cone_array_msg.cone_array.push_back(cone_message);
  }
  cone_array_msg.header.stamp = msg.header.stamp;
  custom_interfaces::msg::PerceptionOutput perception_output;
  perception_output.header = cone_array_msg.header;
  perception_output.cones = cone_array_msg;
  perception_output.exec_time = 0.0;
  _perception_subscription_callback(perception_output);
}

void PacsimAdapter::_pacsim_velocities_subscription_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped& msg) {
  custom_interfaces::msg::Velocities velocities;
  velocities.header.stamp = msg.header.stamp;
  velocities.velocity_x = msg.twist.twist.linear.x;
  velocities.velocity_y = msg.twist.twist.linear.y;
  velocities.angular_velocity = msg.twist.twist.angular.z;
  velocities.header.stamp = msg.header.stamp;
  for (unsigned int i = 0; i < 9; i++) {
    velocities.covariance[i] = 0.00001;  // Pacsim does not provide covariance, assume small value
                                         // or else shit breaks if used by solver
  }
  _velocities_subscription_callback(velocities);
}

void PacsimAdapter::_pacsim_imu_subscription_callback(const sensor_msgs::msg::Imu& msg) {
  _imu_subscription_callback(msg);
}