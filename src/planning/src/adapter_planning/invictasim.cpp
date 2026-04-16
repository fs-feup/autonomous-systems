#include "adapter_planning/invictasim.hpp"

#include <cstdlib>

#include "common_lib/config_load/config_load.hpp"

InvictaSimAdapter::InvictaSimAdapter(const PlanningParameters& params) : Planning(params) {
  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("planning", "global", "global_config");
  YAML::Node global_config = YAML::LoadFile(global_config_path);

  std::string discipline = global_config["global"]["discipline"].as<std::string>();
  if (const char* env_discipline = std::getenv("AS_DISCIPLINE");
      env_discipline != nullptr && env_discipline[0] != '\0') {
    discipline = env_discipline;
  }

  auto mission_it = common_lib::competition_logic::fsds_to_system.find(discipline);
  if (mission_it != common_lib::competition_logic::fsds_to_system.end()) {
    mission_ = mission_it->second;
  } else {
    mission_ = common_lib::competition_logic::Mission::AUTOCROSS;
    RCLCPP_WARN(this->get_logger(),
                "Planning: unknown invictasim discipline '%s', defaulting to autocross",
                discipline.c_str());
  }

  if (planning_config_.using_simulated_se_) {
    RCLCPP_INFO(this->get_logger(), "Planning: Invictasim using simulator ground truth");
    pose_subscription_ = this->create_subscription<custom_interfaces::msg::Pose>(
        "/invictasim/pose", 10,
        std::bind(&InvictaSimAdapter::pose_callback, this, std::placeholders::_1));
    track_subscription_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "/invictasim/track", 10,
        std::bind(&InvictaSimAdapter::track_callback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(this->get_logger(), "Planning: Invictasim adapter created for discipline '%s'",
              discipline.c_str());
}

void InvictaSimAdapter::pose_callback(const custom_interfaces::msg::Pose& msg) {
  this->vehicle_localization_callback(msg);
}

void InvictaSimAdapter::track_callback(const custom_interfaces::msg::ConeArray& msg) {
  this->track_map_callback(msg);
}

void InvictaSimAdapter::finish() {
  RCLCPP_DEBUG(this->get_logger(), "Planning: finish is undefined for Invictasim");
}
