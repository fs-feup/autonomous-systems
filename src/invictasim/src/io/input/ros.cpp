#include "io/input/ros.hpp"

RosInputAdapter::RosInputAdapter(const std::shared_ptr<InvictaSim>& simulator)
    : Node("invictasim_input", rclcpp::NodeOptions().use_global_arguments(false)),
      InvictaSimInputAdapter(simulator) {
  control_command_sub_ = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      "/control/command", 10, [this](const custom_interfaces::msg::ControlCommand::SharedPtr msg) {
        common_lib::structures::Wheels throttle;
        throttle.front_left = msg->throttle_fl;
        throttle.front_right = msg->throttle_fr;
        throttle.rear_left = msg->throttle_rl;
        throttle.rear_right = msg->throttle_rr;
        simulator_->set_input(throttle, msg->steering);
      });

  if (!simulator_->get_params().use_simulated_se) {
    slam_map_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "/state_estimation/map", 10,
        [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
          std::vector<common_lib::structures::Cone> cones;
          cones.reserve(msg->cone_array.size());
          for (const auto& cone_msg : msg->cone_array) {
            cones.push_back(common_lib::structures::Cone(
                common_lib::structures::Position{cone_msg.position.x, cone_msg.position.y},
                common_lib::competition_logic::Color::UNKNOWN, cone_msg.confidence,
                msg->header.stamp));
          }
          simulator_->set_external_slam_cones(cones);
        });
  }
}
