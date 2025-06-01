#include "adapter_slam/vehicle.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "common_lib/competition_logic/color.hpp"
#include "ros_node/slam_node.hpp"

VehicleAdapter::VehicleAdapter(const SLAMParameters& params) : SLAMNode(params) {
  _operational_status_subscription_ =
      this->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          [this](const custom_interfaces::msg::OperationalStatus::SharedPtr msg) {
            RCLCPP_DEBUG(this->get_logger(), "Operational status received. Mission: %d - Go: %d",
                         msg->as_mission, msg->go_signal);
            _go_ = msg->go_signal;
            common_lib::competition_logic::Mission previous_mission_ = _mission_;
            _mission_ = common_lib::competition_logic::Mission(msg->as_mission);
            this->_slam_solver_->set_mission(_mission_);
          });
  _finished_client_ = this->create_client<std_srvs::srv::Trigger>("/as_srv/mission_finished");

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