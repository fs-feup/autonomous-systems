#pragma once

#include <atomic>
#include <map>

#include "custom_interfaces/msg/aero_forces.hpp"
#include "custom_interfaces/msg/battery_state.hpp"
#include "custom_interfaces/msg/execution_times.hpp"
#include "custom_interfaces/msg/motor_state.hpp"
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
  // Used for spinning wheels visualization
  double wheel_spin_fl_ = 0.0;
  double wheel_spin_fr_ = 0.0;
  double wheel_spin_rl_ = 0.0;
  double wheel_spin_rr_ = 0.0;
  double last_visualization_stamp_sec_ = -1.0;

  // Publishing timers and frequencies
  std::atomic<bool> running_;
  std::map<std::string, int> publish_frequencies_;
  std::map<int, rclcpp::TimerBase::SharedPtr> frequency_timers_;

  // Snapshot caches of data to be published
  VehicleModelSnapshot vehicle_model_snapshot_cache_;
  ExecutionTimesSnapshot execution_times_snapshot_cache_;

  /**
   * @brief Setup ROS timers for periodic publishing based on configured frequencies.
   */
  void setup_timers();

  /**
   * @brief Callback for timer ticks at different frequencies. Determines which data groups to
   * publish based on the tick frequency.
   */
  void on_frequency_tick(int frequency_hz);

  /**
   * @brief Check if a given data group should be published at the specified frequency.
   */
  bool publishes_at(const std::string& group, int frequency_hz) const;

  /**
   * @brief Publish tire forces, slip ratio, and slip angle for all wheels.
   */
  void publish_tire_group();

  /**
   * @brief Publish motor state information.
   */
  void publish_motor_group();

  /**
   * @brief Publish battery state information.
   */
  void publish_battery_group();

  /**
   * @brief Publish differential state information.
   */
  void publish_differential_group();

  /**
   * @brief Publish aerodynamic forces information.
   */
  void publish_aero_group();

  /**
   * @brief Publish wheel vertical load information.
   */
  void publish_load_group();

  /**
   * @brief Publish vehicle status information.
   */
  void publish_status_group();

  /**
   * @brief Publish simulation execution times information.
   */
  void publish_execution_times_group();

  /**
   * @brief Publish visualization markers for the ground and vehicle.
   */
  void publish_visualization_group();

  /**
   * @brief Refresh the cached vehicle model snapshot with the latest data from the simulator.
   */
  void refresh_vehicle_model_snapshot();

  /**
   * @brief Refresh the cached execution times snapshot with the latest data from the simulator.
   */
  void refresh_execution_times_snapshot();

  /**
   * @brief Convert the given Wheels data into a WheelScalars ROS message, including the provided
   * timestamp.
   */
  custom_interfaces::msg::WheelScalars to_wheels_msg(const common_lib::structures::Wheels& wheels,
                                                     const rclcpp::Time& stamp) const;

  /**
   * @brief Publish visualization markers for the ground.
   * @param marker_array Marker array to populate with visualization markers.
   * @param stamp Current ROS time for timestamping markers.
   */
  void publish_ground_marker(visualization_msgs::msg::MarkerArray& marker_array,
                             const rclcpp::Time& stamp) const;

  /**
   * @brief Publish visualization markers of the car body from mesh.
   * @param marker_array Marker array to populate with visualization markers.
   * @param stamp Current ROS time for timestamping markers.
   */
  void publish_body_marker(visualization_msgs::msg::MarkerArray& marker_array,
                           const rclcpp::Time& stamp) const;

  /**
   * @brief Publish visualization markers for the spinning wheels, using the current wheel spin
   * values.
   * @param marker_array Marker array to populate with visualization markers.
   * @param stamp Current ROS time for timestamping markers.
   * @param dt Time delta since last visualization update, used for calculating wheel spin
   * increments.
   */
  void publish_wheel_markers(visualization_msgs::msg::MarkerArray& marker_array,
                             const rclcpp::Time& stamp, double dt);

  rclcpp::Publisher<custom_interfaces::msg::TireForces>::SharedPtr
      tire_forces_pub_;  ///< Publisher for tire forces.
  rclcpp::Publisher<custom_interfaces::msg::WheelScalars>::SharedPtr
      tire_slip_ratio_pub_;  ///< Publisher for tire slip ratio.
  rclcpp::Publisher<custom_interfaces::msg::WheelScalars>::SharedPtr
      tire_slip_angle_pub_;  ///< Publisher for tire slip angle.
  rclcpp::Publisher<custom_interfaces::msg::MotorState>::SharedPtr
      motor_pub_;  ///< Publisher for motor state.
  rclcpp::Publisher<custom_interfaces::msg::BatteryState>::SharedPtr
      battery_pub_;  ///< Publisher for battery state.
  rclcpp::Publisher<custom_interfaces::msg::WheelScalars>::SharedPtr
      differential_pub_;  ///< Publisher for differential state.
  rclcpp::Publisher<custom_interfaces::msg::AeroForces>::SharedPtr
      aero_pub_;  ///< Publisher for aerodynamic forces.
  rclcpp::Publisher<custom_interfaces::msg::WheelScalars>::SharedPtr
      load_pub_;  ///< Publisher for wheel vertical loads.
  rclcpp::Publisher<custom_interfaces::msg::VehicleStateVector>::SharedPtr
      status_pub_;  ///< Publisher for vehicle status.
  rclcpp::Publisher<custom_interfaces::msg::ExecutionTimes>::SharedPtr
      execution_times_pub_;  ///< Publisher for simulation execution timings.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_ground_pub_;  ///< Publisher for ground visualization markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_vehicle_pub_;  ///< Publisher for vehicle visualization markers.
};
