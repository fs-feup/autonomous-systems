#include "adapter_slam/vehicle.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "common_lib/competition_logic/color.hpp"
#include "ros_node/slam_node.hpp"
#include "slam_solver/solver_traits/odometry_integrator_trait.hpp"

VehicleAdapter::VehicleAdapter(const SLAMParameters& params) : SLAMNode(params) {
  _operational_status_subscription_ =
      this->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          [this](const custom_interfaces::msg::OperationalStatus::SharedPtr msg) {
            RCLCPP_DEBUG(this->get_logger(), "Operational status received. Mission: %d - Go: %d",
                         msg->as_mission, msg->go_signal);
            _go_ = msg->go_signal;
            _mission_ = common_lib::competition_logic::Mission(msg->as_mission);
            this->_slam_solver_->set_mission(_mission_);
          });
  _finished_client_ = this->create_client<std_srvs::srv::Trigger>("/as_srv/mission_finished");

  RCLCPP_DEBUG(this->get_logger(), "VehicleAdapter initialized, topic for lidar odometry: %s",
               params.lidar_odometry_topic_.c_str());

  this->_lidar_odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      params.lidar_odometry_topic_, 1,
      std::bind(&VehicleAdapter::_lidar_odometry_subscription_callback, this,
                std::placeholders::_1));

  // Create a static map frame
  _tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->get_clock()->now();
  transformStamped.header.frame_id = "map";              // Fixed frame: "map"
  transformStamped.child_frame_id = "vehicle_estimate";  // The child frame: "vehicle"

  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;

  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = 0.0;
  transformStamped.transform.rotation.w = 1.0;

  _tf_static_broadcaster_->sendTransform(transformStamped);
}

// TODO: implement a more complex logic, like the one in inspection node
void VehicleAdapter::finish() {
  _finished_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (future.get()->success) {
          RCLCPP_INFO(this->get_logger(), "Finished signal sent");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to send finished signal");
        }
      });
}

void VehicleAdapter::_lidar_odometry_subscription_callback(const nav_msgs::msg::Odometry& msg) {
  if (this->_mission_ == common_lib::competition_logic::Mission::NONE) {
    return;
  }

  rclcpp::Time start_time = this->get_clock()->now();
  if (auto solver_ptr = std::dynamic_pointer_cast<OdometryIntegratorTrait>(this->_slam_solver_)) {
    common_lib::structures::Pose pose;
    pose.position.x = msg.pose.pose.position.x;
    pose.position.y = msg.pose.pose.position.y;
    tf2::Quaternion quat(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    pose.orientation = yaw;  // Use yaw as the orientation
    pose.timestamp = msg.header.stamp;

    solver_ptr->add_odometry(pose);
    this->_vehicle_pose_ = this->_slam_solver_->get_pose_estimate();
  }

  this->_publish_vehicle_pose();

  rclcpp::Time end_time = this->get_clock()->now();
  this->_execution_times_->at(0) = (end_time - start_time).seconds() * 1000.0;
  std_msgs::msg::Float64MultiArray execution_time_msg;
  execution_time_msg.data = *this->_execution_times_;
  this->_execution_time_publisher_->publish(execution_time_msg);
}