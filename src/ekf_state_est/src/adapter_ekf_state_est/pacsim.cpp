#include "adapter_ekf_state_est/pacsim.hpp"

#include "common_lib/competition_logic/color.hpp"
#include "custom_interfaces/msg/cone.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "pacsim/msg/wheels.hpp"
#include "ros_node/se_node.hpp"

PacsimAdapter::PacsimAdapter(std::shared_ptr<SENode> se_node) : Adapter(se_node) {
  this->_imu_subscription_ = this->node_->create_subscription<sensor_msgs::msg::Imu>(
      "/pacsim/imu/cog_imu", 3,
      std::bind(&Adapter::imu_subscription_callback, this, std::placeholders::_1));

  this->_pacsim_wheel_speeds_subscription_.subscribe(this->node_, "/pacsim/wheelspeeds");
  this->_steering_angle_subscription_.subscribe(this->node_, "/pacsim/steeringFront");
  const WheelSteerPolicy policy(10);
  this->_sync_ = std::make_shared<message_filters::Synchronizer<WheelSteerPolicy>>(
      policy, _pacsim_wheel_speeds_subscription_, _steering_angle_subscription_);
  this->_sync_->registerCallback(&PacsimAdapter::wheel_speeds_subscription_callback, this);

  if (this->node_->_use_simulated_perception_) {
    this->_perception_detections_subscription_ =
        this->node_->create_subscription<pacsim::msg::PerceptionDetections>(
            "/pacsim/perception/livox_front/landmarks", 10,
            std::bind(&PacsimAdapter::perception_detections_subscription_callback, this,
                      std::placeholders::_1));
  }

  this->_finished_client_ =
      this->node_->create_client<std_srvs::srv::Empty>("/pacsim/finish_signal");
}

void PacsimAdapter::wheel_speeds_subscription_callback(
    const pacsim::msg::Wheels& wheels_msg, const pacsim::msg::StampedScalar& steering_angle_msg) {
  this->node_->_wheel_speeds_subscription_callback(wheels_msg.rl, wheels_msg.fl, wheels_msg.rr,
                                                   wheels_msg.fr, steering_angle_msg.value,
                                                   steering_angle_msg.stamp);
}

void PacsimAdapter::finish() {
  this->_finished_client_->async_send_request(
      std::make_shared<std_srvs::srv::Empty::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
        RCLCPP_INFO(this->node_->get_logger(), "Finished signal sent");
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
  this->node_->_perception_subscription_callback(cone_array_msg);
}