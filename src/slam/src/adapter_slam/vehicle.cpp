#include "adapter_slam/vehicle.hpp"

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
            _mission_ = common_lib::competition_logic::Mission(msg->as_mission);
          });

  _finished_client_ = this->create_client<std_srvs::srv::Trigger>("/as_srv/mission_finished");
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