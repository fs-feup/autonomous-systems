#include "adapter_slam/pacsim.hpp"

#include "common_lib/competition_logic/color.hpp"
#include "custom_interfaces/msg/cone.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "ros_node/slam_node.hpp"

PacsimAdapter::PacsimAdapter() : SLAMNode() {
  if (_use_simulated_perception_) {
    this->_perception_detections_subscription_ =
        this->create_subscription<pacsim::msg::PerceptionDetections>(
            "/pacsim/perception/livox_front/landmarks", 1,
            std::bind(&PacsimAdapter::perception_detections_subscription_callback, this,
                      std::placeholders::_1));
  }

  if (_use_simulated_velocities_) {
    this->_velocities_subscription_ =
        this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/pacsim/velocities", 1,
            std::bind(&PacsimAdapter::velocities_subscription_callback, this,
                      std::placeholders::_1));
  }

  this->_finished_client_ = this->create_client<std_srvs::srv::Empty>("/pacsim/finish_signal");
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
  _perception_subscription_callback(cone_array_msg);
}

void PacsimAdapter::velocities_subscription_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped& msg) {
  custom_interfaces::msg::Velocities velocities;
  velocities.velocity_x = msg.twist.twist.linear.x;
  velocities.velocity_y = msg.twist.twist.linear.y;
  velocities.angular_velocity = msg.twist.twist.angular.z;
  _velocities_subscription_callback(velocities);
}