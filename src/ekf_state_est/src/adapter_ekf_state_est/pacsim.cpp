#include "adapter_ekf_state_est/pacsim.hpp"

#include "common_lib/competition_logic/color.hpp"
#include "custom_interfaces/msg/cone.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "pacsim/msg/wheels.hpp"
#include "ros_node/se_node.hpp"

PacsimAdapter::PacsimAdapter(bool use_odometry, bool use_simulated_perception,
                             std::string motion_model_name, std::string data_assocation_model_name,
                             float sml_da_curvature, float sml_initial_limit,
                             float observation_noise, float wheel_speed_sensor_noise,
                             float data_association_limit_distance)
    : SENode(use_odometry, use_simulated_perception, motion_model_name, data_assocation_model_name,
             sml_da_curvature, sml_initial_limit, observation_noise, wheel_speed_sensor_noise,
             data_association_limit_distance) {
  this->_imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/pacsim/imu/cog_imu", 3,
      std::bind(&PacsimAdapter::imu_subscription_callback, this, std::placeholders::_1));

  this->_pacsim_wheel_speeds_subscription_.subscribe(this, "/pacsim/wheelspeeds");
  this->_steering_angle_subscription_.subscribe(this, "/pacsim/steeringFront");
  const WheelSteerPolicy policy(1);
  this->_sync_ = std::make_shared<message_filters::Synchronizer<WheelSteerPolicy>>(
      policy, _pacsim_wheel_speeds_subscription_, _steering_angle_subscription_);
  this->_sync_->registerCallback(&PacsimAdapter::wheel_speeds_subscription_callback, this);

  if (this->_use_simulated_perception_) {
    this->_perception_detections_subscription_ =
        this->create_subscription<pacsim::msg::PerceptionDetections>(
            "/pacsim/perception/livox_front/landmarks", 1,
            std::bind(&PacsimAdapter::perception_detections_subscription_callback, this,
                      std::placeholders::_1));
  }

  this->_finished_client_ = this->create_client<std_srvs::srv::Empty>("/pacsim/finish_signal");
}

void PacsimAdapter::wheel_speeds_subscription_callback(
    const pacsim::msg::Wheels& wheels_msg, const pacsim::msg::StampedScalar& steering_angle_msg) {
  this->_wheel_speeds_subscription_callback(wheels_msg.rl, wheels_msg.fl, wheels_msg.rr,
                                            wheels_msg.fr, steering_angle_msg.value,
                                            steering_angle_msg.stamp);
}

void PacsimAdapter::finish() {
  this->_finished_client_->async_send_request(
      std::make_shared<std_srvs::srv::Empty::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
        RCLCPP_INFO(this->get_logger(), "Finished signal sent");
      });
}

void PacsimAdapter::perception_detections_subscription_callback(
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
  this->_perception_subscription_callback(cone_array_msg);
}