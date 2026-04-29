#include "adapter_planning/invictasim.hpp"

InvictaSimAdapter::InvictaSimAdapter(const PlanningParameters& params) : Planning(params) {
  this->mission_subscription_ =
      this->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/invictasim/operational_status", 10,
          std::bind(&InvictaSimAdapter::mission_state_callback, this, std::placeholders::_1));

  if (params.planning_using_simulated_se_) {
    vehicle_localization_sub_ = create_subscription<custom_interfaces::msg::Pose>(
        "/invictasim/state_estimation/vehicle_pose", 10,
        std::bind(&InvictaSimAdapter::pose_callback, this, std::placeholders::_1));

    track_map_sub_ = create_subscription<custom_interfaces::msg::ConeArray>(
        "/invictasim/state_estimation/map", 10,
        std::bind(&InvictaSimAdapter::map_callback, this, std::placeholders::_1));

    lap_counter_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/invictasim/state_estimation/lap_counter", 10,
        std::bind(&InvictaSimAdapter::lap_counter_callback, this, std::placeholders::_1));
  }
}

void InvictaSimAdapter::map_callback(const custom_interfaces::msg::ConeArray& msg) {
  this->track_map_callback(msg);
}

void InvictaSimAdapter::pose_callback(const custom_interfaces::msg::Pose& msg) {
  this->vehicle_localization_callback(msg);
}

void InvictaSimAdapter::lap_counter_callback(const std_msgs::msg::Float64& msg) {
  this->lap_counter_ = static_cast<int>(msg.data);
}

void InvictaSimAdapter::mission_state_callback(
    const custom_interfaces::msg::OperationalStatus& msg) {
  this->mission_ = common_lib::competition_logic::Mission(msg.as_mission);
}

void InvictaSimAdapter::finish() { RCLCPP_INFO(this->get_logger(), "Planning finished..."); }