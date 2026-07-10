#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simulator/invictasim.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class TuningEvaluator : public rclcpp::Node {
public:
  TuningEvaluator(const std::shared_ptr<InvictaSim>& simulator,
                  const InvictaSimParameters& params);
  ~TuningEvaluator() override;

private:
  struct PoseSample {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  struct VelocitySample {
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    double yaw_rate = 0.0;
  };

  struct RunningRmse {
    double sum_squares = 0.0;
    int count = 0;

    double update(double value);
  };

  static double normalize_angle(double angle);
  static PoseSample transform_pose_to_map(const PoseSample& pose, const PoseSample& source_origin,
                                          const PoseSample& target_origin);

  void control_callback(const custom_interfaces::msg::ControlCommand::SharedPtr msg);
  void real_pose_callback(const custom_interfaces::msg::Pose::SharedPtr msg);
  void real_velocity_callback(const custom_interfaces::msg::Velocities::SharedPtr msg);
  void real_fl_rpm_callback(const custom_interfaces::msg::WheelRPM::SharedPtr msg);
  void real_fr_rpm_callback(const custom_interfaces::msg::WheelRPM::SharedPtr msg);
  void real_motor_rpm_callback(const custom_interfaces::msg::WheelRPM::SharedPtr msg);
  void on_timer();
  void try_start_evaluation();

  void publish_pose(const PoseSample& pose, bool real);
  void publish_float(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& publisher,
                     double value);
  void publish_comparison_visualization(const PoseSample& real_pose, const PoseSample& sim_pose,
                                        double position_error);
  void open_csv();
  void write_summary_csv();
  void write_csv_row(const PoseSample& real_pose, const PoseSample& sim_pose,
                     double position_error, double heading_error);

  std::shared_ptr<InvictaSim> simulator_;
  std::filesystem::path output_directory_;

  rclcpp::TimerBase::SharedPtr timer_;

  custom_interfaces::msg::Pose latest_real_pose_;
  custom_interfaces::msg::Velocities latest_real_velocity_;
  bool has_real_pose_ = false;
  bool has_real_velocity_ = false;
  bool has_real_fl_rpm_ = false;
  bool has_real_fr_rpm_ = false;
  bool has_real_motor_rpm_ = false;
  bool has_control_command_ = false;
  double latest_real_fl_rpm_ = 0.0;
  double latest_real_fr_rpm_ = 0.0;
  double latest_real_motor_rpm_ = 0.0;

  PoseSample real_origin_;
  PoseSample sim_origin_;
  bool started_ = false;
  bool output_samples_ = false;
  std::chrono::steady_clock::time_point start_time_;

  RunningRmse position_rmse_;
  RunningRmse heading_rmse_;
  RunningRmse velocity_rmse_;
  RunningRmse yaw_rate_rmse_;
  RunningRmse front_wheel_rpm_rmse_;
  RunningRmse motor_rpm_rmse_;

  std::ofstream csv_file_;

  rclcpp::Subscription<custom_interfaces::msg::ControlCommand>::SharedPtr control_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr real_pose_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Velocities>::SharedPtr real_velocity_sub_;
  rclcpp::Subscription<custom_interfaces::msg::WheelRPM>::SharedPtr real_fl_rpm_sub_;
  rclcpp::Subscription<custom_interfaces::msg::WheelRPM>::SharedPtr real_fr_rpm_sub_;
  rclcpp::Subscription<custom_interfaces::msg::WheelRPM>::SharedPtr real_motor_rpm_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_rmse_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_rmse_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_rmse_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_rmse_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_wheel_rpm_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_wheel_rpm_rmse_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_rpm_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_rpm_rmse_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr real_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sim_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
};
