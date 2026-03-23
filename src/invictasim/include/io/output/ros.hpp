#pragma once

#include <atomic>
#include <map>

#include "custom_interfaces/msg/aero_forces.hpp"
#include "custom_interfaces/msg/powertrain_state.hpp"
#include "custom_interfaces/msg/tire_forces.hpp"
#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "custom_interfaces/msg/wheel_scalars.hpp"
#include "io/output/output_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/**
 * @brief ROS-based simulator output adapter.
 */
class RosOutputAdapter : public rclcpp::Node, public InvictaSimOutputAdapter {
public:
  /**
   * @brief Construct a new RosOutputAdapter.
   * @param simulator Simulator instance.
   */
  explicit RosOutputAdapter(const std::shared_ptr<InvictaSim>& simulator);

  /**
   * @brief Start adapter loop.
   */
  void run() override;

  /**
   * @brief Stop adapter loop.
   */
  void stop() override;

private:
  void setup_timers();
  void on_frequency_tick(int frequency_hz);
  bool publishes_at(const std::string& group, int frequency_hz) const;
  void publish_tire_group();
  void publish_powertrain_group();
  void publish_aero_group();
  void publish_load_group();
  void publish_status_group();
  void publish_visualization_group();

  void refresh_tire_snapshot();
  void refresh_powertrain_snapshot();
  void refresh_aero_snapshot();
  void refresh_load_snapshot();
  void refresh_status_snapshot();
  custom_interfaces::msg::WheelScalars to_wheels_msg(const common_lib::structures::Wheels& wheels,
                                                     const rclcpp::Time& stamp) const;

  std::atomic<bool> running_;
  std::map<std::string, int> publish_frequencies_;
  std::map<int, rclcpp::TimerBase::SharedPtr> frequency_timers_;

  TireSnapshot tire_snapshot_cache_;
  PowertrainSnapshot powertrain_snapshot_cache_;
  AeroSnapshot aero_snapshot_cache_;
  LoadSnapshot load_snapshot_cache_;
  StatusSnapshot status_snapshot_cache_;

  rclcpp::Publisher<custom_interfaces::msg::TireForces>::SharedPtr
      tire_forces_pub_;  ///< Publisher for tire forces.
  rclcpp::Publisher<custom_interfaces::msg::WheelScalars>::SharedPtr
      tire_slip_ratio_pub_;  ///< Publisher for tire slip ratio.
  rclcpp::Publisher<custom_interfaces::msg::WheelScalars>::SharedPtr
      tire_slip_angle_pub_;  ///< Publisher for tire slip angle.
  rclcpp::Publisher<custom_interfaces::msg::PowertrainState>::SharedPtr
      powertrain_pub_;  ///< Publisher for grouped powertrain state.
  rclcpp::Publisher<custom_interfaces::msg::AeroForces>::SharedPtr
      aero_forces_pub_;  ///< Publisher for aerodynamic forces.
  rclcpp::Publisher<custom_interfaces::msg::WheelScalars>::SharedPtr
      wheel_load_pub_;  ///< Publisher for per-wheel vertical load.
  rclcpp::Publisher<custom_interfaces::msg::VehicleStateVector>::SharedPtr
      vehicle_state_pub_;  ///< Publisher for the vehicle state message.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_pub_;  ///< Publisher for foxglove visualization marker.
};
