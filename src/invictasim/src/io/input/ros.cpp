#include "io/input/ros.hpp"

#include <yaml-cpp/yaml.h>

#include "common_lib/communication/interfaces.hpp"
#include "common_lib/config_load/config_load.hpp"
#include "config/yaml_helpers.hpp"

RosInputAdapter::Topics RosInputAdapter::load_topics(const std::string& config_file) {
  using invictasim::config::load_optional;

  Topics topics;

  const std::string full_path =
      common_lib::config_load::get_config_yaml_path("invictasim", "invictasim/input", config_file);
  const YAML::Node config = YAML::LoadFile(full_path);

  const YAML::Node topic_config = config["topics"];
  load_optional(topic_config, "control_command", topics.control_command);
  load_optional(topic_config, "path", topics.path);
  load_optional(topic_config, "slam_map", topics.slam_map);
  load_optional(topic_config, "perception_cones", topics.perception_cones);

  load_optional(config, "queue_depth", topics.queue_depth);

  return topics;
}

RosInputAdapter::RosInputAdapter(const std::shared_ptr<InvictaSim>& simulator,
                                 const std::string& config_file)
    : Node("invictasim_input", rclcpp::NodeOptions().use_global_arguments(false)),
      InvictaSimInputAdapter(simulator) {
  const Topics topics = load_topics(config_file);

  control_command_sub_ = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      topics.control_command, topics.queue_depth,
      [this](const custom_interfaces::msg::ControlCommand::SharedPtr msg) {
        common_lib::structures::Wheels throttle;
        throttle.front_left = msg->throttle_fl;
        throttle.front_right = msg->throttle_fr;
        throttle.rear_left = msg->throttle_rl;
        throttle.rear_right = msg->throttle_rr;
        simulator_->set_input(throttle, msg->steering);
      });

  path_sub_ = this->create_subscription<custom_interfaces::msg::PathPointArray>(
      topics.path, topics.queue_depth,
      [this](const custom_interfaces::msg::PathPointArray::SharedPtr msg) {
        simulator_->set_path_points(
            common_lib::communication::path_point_array_from_ci_vector(*msg));
      });

  if (!simulator_->get_params().use_simulated_se) {
    slam_map_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        topics.slam_map, topics.queue_depth,
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
        topics.perception_cones, topics.queue_depth,
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
