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

  const std::string path_topic = simulator_->get_params().use_simulated_planning
                                     ? "/path_planning/mock_path"
                                     : "/path_planning/path";
  path_sub_ = this->create_subscription<custom_interfaces::msg::PathPointArray>(
      path_topic, 10, [this](const custom_interfaces::msg::PathPointArray::SharedPtr msg) {
        std::vector<PathPointSnapshot> path_points;
        path_points.reserve(msg->pathpoint_array.size());
        for (const auto& path_point_msg : msg->pathpoint_array) {
          PathPointSnapshot path_point;
          path_point.position = {path_point_msg.x, path_point_msg.y};
          path_point.velocity = path_point_msg.v;
          path_points.push_back(path_point);
        }
        simulator_->set_path_points(path_points);
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
                common_lib::competition_logic::Color::RED, cone_msg.confidence, msg->header.stamp));
          }
          simulator_->set_external_slam_cones(cones);
        });
  }

  if (!simulator_->get_params().use_simulated_perception) {
    perception_sub_ = this->create_subscription<custom_interfaces::msg::PerceptionOutput>(
        "/perception/cones", 10,
        [this](const custom_interfaces::msg::PerceptionOutput::SharedPtr msg) {
          std::vector<common_lib::structures::Cone> cones;
          cones.reserve(msg->cones.cone_array.size());
          for (const auto& cone_msg : msg->cones.cone_array) {
            cones.push_back(common_lib::structures::Cone(
                common_lib::structures::Position{cone_msg.position.x, cone_msg.position.y},
                common_lib::competition_logic::Color::GREEN, cone_msg.confidence,
                msg->header.stamp));
          }
          simulator_->set_external_perception_cones(cones);
        });
  }
}
