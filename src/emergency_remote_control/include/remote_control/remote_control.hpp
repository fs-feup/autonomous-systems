#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace remote_control {

struct EmergencyControlPayload {
  double steering_rad{0.0};
  double throttle{0.0};
  bool e_ebs{false};
  bool take_control{false};
  int64_t sent_at_ms{0};
};

struct ParseResult {
  bool valid{false};
  EmergencyControlPayload payload;
  std::string error;
};

struct StopPidConfig {
  double kp{0.0002};
  double ki{0.0};
  double kd{0.0};
  double output_min{-0.4};
  double output_max{0.4};
  double integral_min{-1.0};
  double integral_max{1.0};
};

class StopPid {
 public:
  explicit StopPid(const StopPidConfig& config);

  double update(double error, double dt_seconds);
  void reset();

 private:
  StopPidConfig config_;
  double integral_{0.0};
  double previous_error_{0.0};
  bool has_previous_error_{false};
};

ParseResult parse_emergency_control_message(const std::string& message);
double clamp_control_value(double value, double min_value, double max_value);
custom_interfaces::msg::ControlCommand make_control_command(double steering_rad,
                                                            double rear_throttle);

class WebsocketServer {
 public:
  using MessageCallback = std::function<void(const std::string&)>;
  using EventCallback = std::function<void(const std::string&)>;

  WebsocketServer(std::string host, uint16_t port, int idle_timeout_ms,
                  MessageCallback message_callback,
                  EventCallback event_callback);
  ~WebsocketServer();

  bool start();
  void stop();

 private:
  class Impl;

  std::unique_ptr<Impl> impl_;
};

class RemoteControlNode : public rclcpp::Node {
 public:
  RemoteControlNode();
  ~RemoteControlNode() override;

 private:
  struct State {
    EmergencyControlPayload payload;
    rclcpp::Time last_message_time;
    bool has_message{false};
    bool stale{true};
    bool ebs_latched{false};
    double motor_rpm{0.0};
    rclcpp::Time last_pid_time;
    bool has_pid_time{false};
  };

  void handle_websocket_message(const std::string& message);
  void handle_websocket_event(const std::string& event);
  void motor_rpm_callback(const custom_interfaces::msg::WheelRPM& msg);
  void command_timer_callback();
  void publish_remote_state(bool ebs, bool take_control);
  void publish_manual_command(const EmergencyControlPayload& payload);
  void publish_ebs_command(State& state, const rclcpp::Time& now);
  void log_state_transition(bool ebs, bool take_control, bool stale);

  std::mutex state_mutex_;
  State state_;

  std::unique_ptr<WebsocketServer> websocket_server_;
  StopPid stop_pid_;

  std::string websocket_host_;
  int websocket_port_{4321};
  int websocket_idle_timeout_ms_{0};
  int stale_timeout_ms_{500};
  int command_period_ms_{25};
  double steering_min_rad_{-0.335};
  double steering_max_rad_{0.335};
  double throttle_min_{-1.0};
  double throttle_max_{1.0};
  double velocity_error_min_{-2000.0};
  double velocity_error_max_{2000.0};
  double ebs_stop_rpm_tolerance_{50.0};

  bool last_published_ebs_{false};
  bool last_published_take_control_{false};
  bool last_logged_stale_{true};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remote_ebs_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remote_take_control_pub_;
  rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr control_command_pub_;
  rclcpp::Subscription<custom_interfaces::msg::WheelRPM>::SharedPtr motor_rpm_sub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};

}  // namespace remote_control
