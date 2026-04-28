#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <cmath>
#include <map>
#include <memory>
#include <set>

#include "common_lib/competition_logic/color.hpp"
#include "common_lib/competition_logic/mission_logic.hpp"
#include "common_lib/structures/cone.hpp"
#include "custom_interfaces/msg/aero_forces.hpp"
#include "custom_interfaces/msg/battery_state.hpp"
#include "custom_interfaces/msg/cone.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/execution_times.hpp"
#include "custom_interfaces/msg/motor_state.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/perception_output.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"
#include "custom_interfaces/msg/tire_forces.hpp"
#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "custom_interfaces/msg/wheel_scalars.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "io/output/output_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/**
 * @brief ROS-based simulator output adapter.
 */
class RosOutputAdapter : public rclcpp::Node, public InvictaSimOutputAdapter {
public:
  /**
   * @brief Construct a new RosOutputAdapter.
   * @param simulator Simulator instance.
   * @param config_file Config file name.
   */
  explicit RosOutputAdapter(const std::shared_ptr<InvictaSim>& simulator,
                            const std::string& config_file);

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
  std::map<int, rclcpp::TimerBase::SharedPtr> frequency_timers_;
  std::map<std::string, int> vehicle_model_publish_frequencies_;
  std::map<std::string, int> visualization_publish_frequencies_;
  std::map<std::string, int> sensors_publish_frequencies_;
  std::map<std::string, int> map_publish_frequencies_;
  std::map<std::string, int> vehicle_state_publish_frequencies_;
  int execution_time_frequency_ = 0;

  // Snapshot caches of data to be published
  VehicleModelSnapshot vehicle_model_snapshot_cache_;
  ExecutionTimesSnapshot execution_times_snapshot_cache_;
  MapSnapshot map_snapshot_cache_;
  SensorsSnapshot sensors_snapshot_cache_;
  VehicleStateSnapshot vehicle_state_snapshot_cache_;

  // Topic frequency
  std::unordered_map<std::string, int> topic_frequencies_;
  std::unordered_map<int, std::vector<std::function<void()>>> frequency_callbacks_;

  // Topic frequency setup functions
  void load_publish_frequencies(const std::string& config_file);
  void map_callbacks();
  void setup_timers();
  void on_frequency_tick(int frequency_hz);
  void load_group_from_yaml(const YAML::Node& config, const std::string& group_name);
  void register_pub_helper(const std::string& topic, std::function<void()> func);

  // Update snapshot caches with latest data from simulator
  void refresh_vehicle_model_snapshot();
  void refresh_execution_times_snapshot();
  void refresh_map_snapshot();
  void refresh_sensors_snapshot();
  void refresh_vehicle_state_snapshot();

  // Vehicle model
  void publish_vm_tire();
  void publish_vm_battery();
  void publish_vm_motor();
  void publish_vm_transmission();
  void publish_vm_aero();
  void publish_vm_status();

  // Visualization
  void publish_visualization_ground();
  void publish_visualization_gt_cones();
  void publish_visualization_slam_cones();
  void publish_visualization_car();
  void publish_visualization_perception_cones();

  // Sensors
  void publish_sensors_imu();
  void publish_sensors_wheel_speed();
  void publish_sensors_resolver();
  void publish_sensors_steering();

  // Map
  void publish_map_ground_truth();
  void publish_state_estimation_map();
  void publish_perception_cones();

  // Vehicle state (for state estimation, SLAM, planning pipelines)
  void publish_state_estimation_pose();
  void publish_state_estimation_velocities();
  void publish_operational_status();

  // Execution time
  void publish_execution_time();

  // Input commands
  void publish_input();

  // Helper functions for message conversions and visualization
  custom_interfaces::msg::WheelScalars to_wheels_msg(const common_lib::structures::Wheels& wheels,
                                                     const rclcpp::Time& stamp) const;

  // Visualization marker publishing helper functions
  visualization_msgs::msg::MarkerArray convert_cone_array_to_markers(
      std::vector<common_lib::structures::Cone>& cone_array, const rclcpp::Time& stamp) const;
  void add_body_marker(visualization_msgs::msg::MarkerArray& marker_array,
                       const rclcpp::Time& stamp) const;
  void add_wheel_markers(visualization_msgs::msg::MarkerArray& marker_array,
                         const rclcpp::Time& stamp, double dt);
  void add_vehicle_transform();

  // ROS publishers
  std::unique_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster_;  ///< Vehicle pose TF publisher, used for having a car perspective.
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
      transmission_pub_;  ///< Publisher for transmission state.
  rclcpp::Publisher<custom_interfaces::msg::AeroForces>::SharedPtr
      aero_pub_;  ///< Publisher for aerodynamic forces.
  rclcpp::Publisher<custom_interfaces::msg::VehicleStateVector>::SharedPtr
      status_pub_;  ///< Publisher for vehicle status.
  rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr
      input_command_pub_;  ///< Publisher for current input command.
  rclcpp::Publisher<custom_interfaces::msg::ExecutionTimes>::SharedPtr
      execution_times_pub_;  ///< Publisher for simulation execution timings.
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr
      map_pub_;  ///< Publisher for the loaded ground truth map.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_ground_pub_;  ///< Publisher for ground visualization markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_vehicle_pub_;  ///< Publisher for vehicle visualization markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_gt_cones_pub_;  ///< Publisher for ground-truth cones visualization markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_slam_cones_pub_;  ///< Publisher for SLAM cones visualization markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_perception_cones_pub_;  ///< Publisher for perception cones visualization
                                            ///< markers.

  // Compatibility publishers for other nodes (ground-truth topics expected by adapters)
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr free_accel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr angular_vel_pub_;
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr vehicle_fl_rpm_pub_;
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr vehicle_fr_rpm_pub_;
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr vehicle_motor_rpm_pub_;
  rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr steering_pub_;
  rclcpp::Publisher<custom_interfaces::msg::PerceptionOutput>::SharedPtr perception_pub_;
  rclcpp::Publisher<custom_interfaces::msg::Velocities>::SharedPtr velocities_pub_;
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr state_map_pub_;
  rclcpp::Publisher<custom_interfaces::msg::OperationalStatus>::SharedPtr operational_status_pub_;
  rclcpp::Publisher<custom_interfaces::msg::Pose>::SharedPtr vehicle_pose_pub_;
};
