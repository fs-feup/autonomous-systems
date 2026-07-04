#include "adapter/invictasim.hpp"

InvictasimAdapter::InvictasimAdapter(const ControlParameters &params)
    : ControlNode(params),
      go_sub_(create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/invictasim/operational_status", 10,
          std::bind(&InvictasimAdapter::go_signal_callback, this, std::placeholders::_1))),
      control_pub_(
          create_publisher<custom_interfaces::msg::ControlCommand>("/control/command", 10)) {
  if (this->params_.using_simulated_slam_) {
    this->pose_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
        "/invictasim/state_estimation/vehicle_pose", 1,
        std::bind(&InvictasimAdapter::pose_callback, this, std::placeholders::_1));
  }

  if (this->params_.using_simulated_velocities_) {
    this->velocities_sub_ = this->create_subscription<custom_interfaces::msg::Velocities>(
        "/invictasim/state_estimation/velocities", 1,
        std::bind(&InvictasimAdapter::velocities_callback, this, std::placeholders::_1));

    this->vehicle_status_sub_ =
        this->create_subscription<custom_interfaces::msg::VehicleStateVector>(
            "/invictasim/vehicle_model/status", 1,
            std::bind(&InvictasimAdapter::vehicle_status_callback, this, std::placeholders::_1));
  }
  RCLCPP_INFO(this->get_logger(), "Invictasim adapter created");
  this->go_signal_ = true;
}

void InvictasimAdapter::publish_command(common_lib::structures::ControlCommand cmd) {
  auto message = custom_interfaces::msg::ControlCommand();

  message.throttle_rr = cmd.throttle_rr;
  message.throttle_rl = cmd.throttle_rl;
  message.throttle_fr = cmd.throttle_fr;
  message.throttle_fl = cmd.throttle_fl;
  message.steering = cmd.steering_angle;

  this->control_pub_->publish(message);
}

void InvictasimAdapter::go_signal_callback(const custom_interfaces::msg::OperationalStatus msg) {
  this->go_signal_ = msg.go_signal;
  if (!(msg.as_mission == common_lib::competition_logic::Mission::TRACKDRIVE) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::AUTOCROSS) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::SKIDPAD) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::ACCELERATION) &&
      !(msg.as_mission == common_lib::competition_logic::Mission::EBS_TEST)) {
    this->go_signal_ = false;
  }
}

void InvictasimAdapter::velocities_callback(const custom_interfaces::msg::Velocities &msg) {
  this->current_state_.velocity_x = msg.velocity_x;
  this->current_state_.velocity_y = msg.velocity_y;
  this->current_state_.yaw_rate = msg.angular_velocity;
  this->vehicle_state_callback(this->current_state_);
}

void InvictasimAdapter::vehicle_status_callback(
    const custom_interfaces::msg::VehicleStateVector &msg) {
  // Longitudinal and lateral acceleration in the car's frame
  this->current_state_.acceleration_x = msg.acceleration_x;
  this->current_state_.acceleration_y = msg.acceleration_y;
  // Steering angle
  this->current_state_.steering_angle = msg.steering_angle;
  // Individual wheel speeds, converted from RPM to rad/s (the unit the MPC expects, see the
  // pacsim adapter)
  this->current_state_.fl_rpm = 2 * M_PI * msg.fl_rpm / 60.0;
  this->current_state_.fr_rpm = 2 * M_PI * msg.fr_rpm / 60.0;
  this->current_state_.rl_rpm = 2 * M_PI * msg.rl_rpm / 60.0;
  this->current_state_.rr_rpm = 2 * M_PI * msg.rr_rpm / 60.0;
  this->vehicle_state_callback(this->current_state_);
}

void InvictasimAdapter::pose_callback(const custom_interfaces::msg::Pose &msg) {
  this->vehicle_pose_callback(msg);
}