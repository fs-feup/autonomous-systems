#include "node/invictasim_node.hpp"

InvictaSimNode::InvictaSimNode(const InvictaSimParameters& params)
    : Node("invictasim_node"), params_(params), sim_time_(0.0) {
  this->vehicle_model_ = vehicle_models_map.at(params_.vehicle_model.c_str())(params);

  RCLCPP_INFO(rclcpp::get_logger("InvictaSim"),
              "Initialized InvictaSim with vehicle model: %s, track: %s, discipline: %s",
              params_.vehicle_model.c_str(), params_.track_name.c_str(),
              params_.discipline.c_str());

  // Publishers
  motor_torque_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/motor/torque", 10);
  battery_current_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/battery/current", 10);
  battery_voltage_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/battery/voltage", 10);
  battery_soc_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/battery/state_of_charge", 10);
  vehicle_state_pub_ = this->create_publisher<custom_interfaces::msg::VehicleStateVector>(
      "/invictasim/vehicle_state", 10);

  // Subscribers
  control_command_sub_ = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      "/control/command", 10, [this](const custom_interfaces::msg::ControlCommand::SharedPtr msg) {
        throttle_input_.front_left = msg->throttle_fl;
        throttle_input_.front_right = msg->throttle_fr;
        throttle_input_.rear_left = msg->throttle_rl;
        throttle_input_.rear_right = msg->throttle_rr;
        steering_input_ = msg->steering;
      });

  init();
}

void InvictaSimNode::init() {
  // Create simulation timer
  auto timer_period = std::chrono::microseconds(
      static_cast<int>((params_.timestep / params_.simulation_speed) * 1000000.0));
  simulation_timer_ =
      this->create_wall_timer(timer_period, std::bind(&InvictaSimNode::simulation_step, this));

  next_loop_time_ = std::chrono::steady_clock::now();
}

void InvictaSimNode::simulation_step() {
  // Synchronize to real-time: wait until it's time for the next step
  auto now = std::chrono::steady_clock::now();
  if (now < next_loop_time_) {
    std::this_thread::sleep_until(next_loop_time_);
  }
  next_loop_time_ = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(params_.timestep / params_.simulation_speed));

  // Advance simulation time
  sim_time_ += params_.timestep;

  // Update vehicle dynamics
  vehicle_model_->step(params_.timestep, throttle_input_, steering_input_);

  // Publish simulation outputs (for now, just publish the simulation time as a placeholder)
  std_msgs::msg::Float64 motor_torque_msg;
  motor_torque_msg.data = vehicle_model_->get_motor_torque();
  motor_torque_pub_->publish(motor_torque_msg);

  std_msgs::msg::Float64 battery_current_msg;
  battery_current_msg.data = vehicle_model_->get_battery_current();
  battery_current_pub_->publish(battery_current_msg);

  std_msgs::msg::Float64 battery_voltage_msg;
  battery_voltage_msg.data = vehicle_model_->get_battery_voltage();
  battery_voltage_pub_->publish(battery_voltage_msg);

  std_msgs::msg::Float64 battery_soc_msg;
  battery_soc_msg.data = vehicle_model_->get_battery_soc();
  battery_soc_pub_->publish(battery_soc_msg);

  // Missing correct information, just to test the publisher
  custom_interfaces::msg::VehicleStateVector vehicle_state_msg;
  vehicle_state_msg.velocity_x = vehicle_model_->get_velocity_x();
  vehicle_state_msg.velocity_y = vehicle_model_->get_velocity_y();
  vehicle_state_msg.yaw_rate = vehicle_model_->get_yaw();
  vehicle_state_msg.acceleration_x = 0.0;
  vehicle_state_msg.acceleration_y = 0.0;
  vehicle_state_msg.steering_angle = steering_input_;
  auto wheels_speed = vehicle_model_->get_wheels_speed();
  vehicle_state_msg.fl_rpm = wheels_speed.front_left * 60.0 / (2.0 * M_PI);  // Convert rad/s to RPM
  vehicle_state_msg.fr_rpm = wheels_speed.front_right * 60.0 / (2.0 * M_PI);
  vehicle_state_msg.rl_rpm = wheels_speed.rear_left * 60.0 / (2.0 * M_PI);
  vehicle_state_msg.rr_rpm = wheels_speed.rear_right * 60.0 / (2.0 * M_PI);
  vehicle_state_pub_->publish(vehicle_state_msg);
}
