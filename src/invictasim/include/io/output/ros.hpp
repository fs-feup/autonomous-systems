#pragma once

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "common_lib/competition_logic/color.hpp"
#include "common_lib/competition_logic/mission_logic.hpp"
#include "common_lib/structures/cone.hpp"
#include "custom_interfaces/msg/aero_forces.hpp"
#include "custom_interfaces/msg/battery_state.hpp"
#include "custom_interfaces/msg/cone.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/control_statistics.hpp"
#include "custom_interfaces/msg/execution_times.hpp"
#include "custom_interfaces/msg/lap_current.hpp"
#include "custom_interfaces/msg/lap_statistics.hpp"
#include "custom_interfaces/msg/lap_summary.hpp"
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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "io/output/output_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float64.hpp"
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
  // Common types
  using TimingLine = std::tuple<common_lib::structures::Position, common_lib::structures::Position>;

  // Runtime state
  std::atomic<bool> running_;

  // Lap summary state
  int last_published_summary_lap_ = 0;
  std::vector<custom_interfaces::msg::LapStatistics> lap_summary_history_;

  // Wheel visualization state
  double wheel_spin_fl_ = 0.0;
  double wheel_spin_fr_ = 0.0;
  double wheel_spin_rl_ = 0.0;
  double wheel_spin_rr_ = 0.0;
  double last_visualization_stamp_sec_ = -1.0;

  // Cached snapshot data
  VehicleModelSnapshot vehicle_model_snapshot_cache_;
  ExecutionTimesSnapshot execution_times_snapshot_cache_;
  MapSnapshot map_snapshot_cache_;
  SensorsSnapshot sensors_snapshot_cache_;
  VehicleStateSnapshot vehicle_state_snapshot_cache_;
  StatisticsSnapshot statistics_snapshot_cache_;

  // Timer and frequency dispatch
  std::map<int, rclcpp::TimerBase::SharedPtr> frequency_timers_;
  std::unordered_map<std::string, int> topic_frequencies_;
  std::unordered_map<int, std::vector<std::function<void(const rclcpp::Time&)>>>
      frequency_callbacks_;

  // Visualization configuration
  struct TrackRibbon {
    std::vector<geometry_msgs::msg::Point> inner;
    std::vector<geometry_msgs::msg::Point> outer;
    bool closed = false;
  };

  struct AsphaltSample {
    geometry_msgs::msg::Point source;
    geometry_msgs::msg::Point target;
    double width = 0.0;
  };

  double cone_standard_radius_ = 0.115;
  double cone_large_radius_ = 0.15;
  double cone_hit_match_distance_ = 0.35;
  double steering_rotation_multiplier_ = 0.0;
  bool visualize_cone_hitboxes_ = false;
  bool use_generated_track_visualization_ = true;
  double track_surface_max_boundary_segment_length_ = 30.0;
  double track_surface_max_loop_segment_length_ = 30.0;
  double track_surface_cone_clearance_ = 1.50;
  double track_surface_curb_width_ = 0.45;
  double track_surface_curb_block_length_ = 0.80;
  double track_surface_curb_z_ = 0.018;
  double track_surface_asphalt_z_ = 0.004;
    double ground_margin_ = 100.0;
    double grass_tile_size_m_ = 1.0;
    mutable std::string grass_mesh_resource_;

  // Visualization marker templates
  visualization_msgs::msg::Marker cone_hitbox_marker_template_;
  visualization_msgs::msg::MarkerArray ground_marker_template_;
  visualization_msgs::msg::Marker body_marker_template_;
  visualization_msgs::msg::Marker steering_marker_template_;
  std::array<visualization_msgs::msg::Marker, 4> wheel_marker_templates_;
  std::vector<visualization_msgs::msg::Marker> car_hitbox_marker_templates_;

  // Timer and frequency setup
  void load_publish_frequencies(const std::string& config_file);
  void map_callbacks();
  void setup_timers();
  void on_frequency_tick(int frequency_hz);
  void load_group_from_yaml(const YAML::Node& config, const std::string& group_name);
  void register_pub_helper(const std::string& topic, std::function<void()> refresh_snapshot,
                           std::function<void(const rclcpp::Time&)> func);

  // Snapshot refresh helpers
  void refresh_vehicle_model_snapshot();
  void refresh_execution_times_snapshot();
  void refresh_map_snapshot();
  void refresh_sensors_snapshot();
  void refresh_vehicle_state_snapshot();
  void refresh_statistics_snapshot();

  // Visualization resource loading
  void load_visualization_resources();
  void load_cone_visualization_resources();
  void load_ground_visualization_resources();
  void load_car_visualization_resources();

  // Vehicle model publishers
  void publish_vm_tire(const rclcpp::Time& stamp);
  void publish_vm_battery(const rclcpp::Time& stamp);
  void publish_vm_motor(const rclcpp::Time& stamp);
  void publish_vm_transmission(const rclcpp::Time& stamp);
  void publish_vm_aero(const rclcpp::Time& stamp);
  void publish_vm_status(const rclcpp::Time& stamp);

  // Visualization publishers
  void publish_visualization_ground(const rclcpp::Time& stamp);
  // Sizes and centres the ground plane on the loaded track, so it is not a huge
  // sheet extending far past anything the car can reach.
  void fit_ground_plane_to_track(visualization_msgs::msg::Marker& ground) const;
  // The plane is sized to the track, but the texture must stay a fixed size in
  // metres, so the UVs depend on the plane. Writes (once per size) a plane mesh
  // whose UV range is the plane size divided by the tile size, and returns its
  // package:// path. The size is in the filename so a changed plane never reuses
  // a cached copy of the old mesh in the viewer.
  std::string ensure_grass_plane_mesh(double size_x, double size_y) const;
  void publish_visualization_gt_cones(const rclcpp::Time& stamp);
  void publish_visualization_slam_cones(const rclcpp::Time& stamp);
  void publish_visualization_car(const rclcpp::Time& stamp);
  void publish_visualization_perception_cones(const rclcpp::Time& stamp);

  // Sensor publishers
  void publish_sensors_imu(const rclcpp::Time& stamp);
  void publish_sensors_wheel_speed(const rclcpp::Time& stamp);
  void publish_sensors_resolver(const rclcpp::Time& stamp);
  void publish_sensors_steering(const rclcpp::Time& stamp);

  // Map and state estimation publishers
  void publish_map_ground_truth(const rclcpp::Time& stamp);
  void publish_state_estimation_map(const rclcpp::Time& stamp);
  void publish_state_estimation_lap_counter();
  void publish_state_estimation_pose(const rclcpp::Time& stamp);
  void publish_state_estimation_state_vector(const rclcpp::Time& stamp);
  void publish_state_estimation_velocities(const rclcpp::Time& stamp);
  void publish_operational_status(const rclcpp::Time& stamp);

  // Execution time publisher
  void publish_execution_time(const rclcpp::Time& stamp);

  // Statistics publishers
  void publish_lap_summary(const rclcpp::Time& stamp);
  void publish_lap_current(const rclcpp::Time& stamp);
  void publish_control_statistics(const rclcpp::Time& stamp);

  // Simulated perception publisher
  void publish_perception_cones(const rclcpp::Time& stamp);

  // Input command publisher
  void publish_input(const rclcpp::Time& stamp);

  // Message conversion helpers
  custom_interfaces::msg::WheelScalars to_wheels_msg(const common_lib::structures::Wheels& wheels,
                                                     const rclcpp::Time& stamp) const;
  std::vector<common_lib::structures::Cone> mark_recently_hit_cones_red(
      std::vector<common_lib::structures::Cone> cones) const;

  // Common visualization helpers
  visualization_msgs::msg::MarkerArray convert_cone_array_to_markers(
      const std::vector<common_lib::structures::Cone>& cone_array, const rclcpp::Time& stamp,
      const std::string& frame_id = "map") const;

  // Ground visualization helpers
  std::vector<TimingLine> make_timing_lines() const;
  std::vector<TimingLine> make_default_timing_lines() const;
  std::vector<TimingLine> make_acceleration_timing_lines() const;
  void add_timing_line_markers(visualization_msgs::msg::MarkerArray& marker_array,
                               double target_cell_length, int row_count, double total_width,
                               double z, double height) const;
  void add_track_surface_markers(visualization_msgs::msg::MarkerArray& marker_array,
                                 const std::vector<common_lib::structures::Cone>& cones,
                                 const rclcpp::Time& stamp) const;

  // Track surface geometry helpers
  visualization_msgs::msg::Marker make_track_triangle_marker(const std::string& ns, int id,
                                                             double z,
                                                             const rclcpp::Time& stamp) const;
  TrackRibbon build_track_ribbon(
      const std::vector<common_lib::structures::Cone>& side_cones,
      const std::vector<common_lib::structures::Cone>& opposite_cones) const;
  void add_track_curb_ribbon(visualization_msgs::msg::Marker& curb_marker,
                             const TrackRibbon& ribbon, const std_msgs::msg::ColorRGBA& curb_red,
                             const std_msgs::msg::ColorRGBA& curb_white) const;
  void add_track_asphalt_from_ribbon(visualization_msgs::msg::Marker& asphalt_marker,
                                     const TrackRibbon& source_ribbon,
                                     const TrackRibbon& target_ribbon,
                                     const std_msgs::msg::ColorRGBA& asphalt_color,
                                     bool reverse_winding) const;
  std_msgs::msg::ColorRGBA make_track_color(float r, float g, float b) const;
  geometry_msgs::msg::Point make_track_point(double x, double y) const;
  std::array<double, 2> track_segment_outward_normal(
      const std::vector<common_lib::structures::Cone>& side_cones,
      const std::vector<common_lib::structures::Cone>& opposite_cones, std::size_t start_i,
      std::size_t end_i) const;
  void add_track_quad(visualization_msgs::msg::Marker& marker, const geometry_msgs::msg::Point& a,
                      const geometry_msgs::msg::Point& b, const geometry_msgs::msg::Point& c,
                      const geometry_msgs::msg::Point& d,
                      const std_msgs::msg::ColorRGBA& color) const;
  double track_path_length(const std::vector<geometry_msgs::msg::Point>& points, bool closed) const;
  geometry_msgs::msg::Point sample_track_path(const std::vector<geometry_msgs::msg::Point>& points,
                                              bool closed, double target_distance) const;
  geometry_msgs::msg::Point closest_point_on_track_path(
      const std::vector<geometry_msgs::msg::Point>& points, bool closed,
      const geometry_msgs::msg::Point& query) const;

  // Vehicle visualization helpers
  std::string get_car_mesh_resource(const std::string& mesh_name) const;
  static std::array<double, 10> load_car_mesh_positions(const YAML::Node& config);
  void add_body_marker(visualization_msgs::msg::MarkerArray& marker_array,
                       const rclcpp::Time& stamp) const;
  void add_steering_marker(visualization_msgs::msg::MarkerArray& marker_array,
                           const rclcpp::Time& stamp) const;
  void add_hitbox_markers(visualization_msgs::msg::MarkerArray& marker_array,
                          const rclcpp::Time& stamp) const;
  void add_wheel_markers(visualization_msgs::msg::MarkerArray& marker_array,
                         const rclcpp::Time& stamp, double dt);
  void add_vehicle_transform(const rclcpp::Time& stamp);

  // ROS transform broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster_;  ///< Vehicle pose TF publisher, used for having a car perspective.

  // Vehicle model ROS publishers
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

  // Input, timing, and map ROS publishers
  rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr
      input_command_pub_;  ///< Publisher for current input command.
  rclcpp::Publisher<custom_interfaces::msg::ExecutionTimes>::SharedPtr
      execution_times_pub_;  ///< Publisher for simulation execution timings.
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr
      map_pub_;  ///< Publisher for the loaded ground truth map.

  // Statistics ROS publishers
  rclcpp::Publisher<custom_interfaces::msg::LapSummary>::SharedPtr
      lap_summary_pub_;  ///< Publisher for completed lap history.
  rclcpp::Publisher<custom_interfaces::msg::LapCurrent>::SharedPtr
      lap_current_pub_;  ///< Publisher for current lap status.
  rclcpp::Publisher<custom_interfaces::msg::ControlStatistics>::SharedPtr
      control_statistics_pub_;  ///< Publisher for controller tracking statistics.

  // Visualization ROS publishers
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

  // Sensor ROS publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      free_accel_pub_;  ///< Publisher for IMU free acceleration.
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      angular_vel_pub_;  ///< Publisher for IMU angular velocity.
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr
      vehicle_fl_rpm_pub_;  ///< Publisher for front left wheel RPM.
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr
      vehicle_fr_rpm_pub_;  ///< Publisher for front right wheel RPM.
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr
      vehicle_motor_rpm_pub_;  ///< Publisher for motor RPM.
  rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr
      steering_pub_;  ///< Publisher for steering angle.

  // Simulated perception and state estimation ROS publishers
  rclcpp::Publisher<custom_interfaces::msg::PerceptionOutput>::SharedPtr
      perception_pub_;  ///< Publisher for simulated perception output.
  rclcpp::Publisher<custom_interfaces::msg::Velocities>::SharedPtr
      velocities_pub_;  ///< Publisher for vehicle velocities.
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr
      state_map_pub_;  ///< Publisher for state estimation map.
  rclcpp::Publisher<custom_interfaces::msg::OperationalStatus>::SharedPtr
      operational_status_pub_;  ///< Publisher for operational status.
  rclcpp::Publisher<custom_interfaces::msg::Pose>::SharedPtr
      vehicle_pose_pub_;  ///< Publisher for vehicle pose.
  rclcpp::Publisher<custom_interfaces::msg::VehicleStateVector>::SharedPtr
      vehicle_state_vector_pub_;  ///< Publisher for vehicle state vector.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      lap_counter_pub_;  ///< Publisher for lap counter.
};
