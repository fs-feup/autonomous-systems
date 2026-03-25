#include "io/output/ros.hpp"

#include <chrono>
#include <cmath>
#include <set>
#include <thread>

#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"

RosOutputAdapter::RosOutputAdapter(const std::shared_ptr<InvictaSim>& simulator)
    : Node("invictasim_output", rclcpp::NodeOptions().use_global_arguments(false)),
      InvictaSimOutputAdapter(simulator),
      running_(false),
      publish_frequencies_(simulator->get_params().publish_frequencies) {
  tire_forces_pub_ = this->create_publisher<custom_interfaces::msg::TireForces>(
      "invictasim/vehicle_model/tire/forces", 10);
  tire_slip_ratio_pub_ = this->create_publisher<custom_interfaces::msg::WheelScalars>(
      "invictasim/vehicle_model/tire/slip_ratio", 10);
  tire_slip_angle_pub_ = this->create_publisher<custom_interfaces::msg::WheelScalars>(
      "invictasim/vehicle_model/tire/slip_angle", 10);
  battery_pub_ = this->create_publisher<custom_interfaces::msg::BatteryState>(
      "invictasim/vehicle_model/battery", 10);
  motor_pub_ = this->create_publisher<custom_interfaces::msg::MotorState>(
      "invictasim/vehicle_model/motor", 10);
  differential_pub_ = this->create_publisher<custom_interfaces::msg::WheelScalars>(
      "invictasim/vehicle_model/differential", 10);
  aero_pub_ = this->create_publisher<custom_interfaces::msg::AeroForces>(
      "invictasim/vehicle_model/aero", 10);
  load_pub_ = this->create_publisher<custom_interfaces::msg::WheelScalars>(
      "invictasim/vehicle_model/load_transfer", 10);
  status_pub_ = this->create_publisher<custom_interfaces::msg::VehicleStateVector>(
      "invictasim/vehicle_model/status", 10);
  execution_times_pub_ = this->create_publisher<custom_interfaces::msg::ExecutionTimes>(
      "invictasim/execution_times", 10);
  visualization_ground_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "invictasim/visualization/ground", 10);
  visualization_vehicle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "invictasim/visualization/vehicle", 10);

  setup_timers();
}

void RosOutputAdapter::run() {
  running_ = true;
  while (running_ && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void RosOutputAdapter::stop() { running_ = false; }

void RosOutputAdapter::setup_timers() {
  std::set<int> unique_frequencies;
  for (const auto& publish_frequency : publish_frequencies_) {
    if (publish_frequency.second > 0) {
      unique_frequencies.insert(publish_frequency.second);
    }
  }

  for (int frequency_hz : unique_frequencies) {
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(frequency_hz)));
    frequency_timers_[frequency_hz] = this->create_wall_timer(period, [this, frequency_hz]() {
      if (!running_) {
        return;
      }
      on_frequency_tick(frequency_hz);
    });
  }
}

bool RosOutputAdapter::publishes_at(const std::string& group, int frequency_hz) const {
  auto group_frequency_it = publish_frequencies_.find(group);
  if (group_frequency_it == publish_frequencies_.end()) {
    return false;
  }
  return group_frequency_it->second == frequency_hz;
}

void RosOutputAdapter::on_frequency_tick(int frequency_hz) {
  // Refresh consolidated snapshot once per tick
  refresh_vehicle_model_snapshot();
  refresh_execution_times_snapshot();

  if (publishes_at("tire", frequency_hz)) {
    publish_tire_group();
  }
  if (publishes_at("motor", frequency_hz)) {
    publish_motor_group();
  }
  if (publishes_at("battery", frequency_hz)) {
    publish_battery_group();
  }
  if (publishes_at("differential", frequency_hz)) {
    publish_differential_group();
  }
  if (publishes_at("aero", frequency_hz)) {
    publish_aero_group();
  }
  if (publishes_at("load_transfer", frequency_hz)) {
    publish_load_group();
  }
  if (publishes_at("status", frequency_hz)) {
    publish_status_group();
  }
  if (publishes_at("execution_times", frequency_hz)) {
    publish_execution_times_group();
  }
  if (publishes_at("visualization", frequency_hz)) {
    publish_visualization_group();
  }
}

void RosOutputAdapter::refresh_vehicle_model_snapshot() {
  vehicle_model_snapshot_cache_ = simulator_->get_vehicle_model_snapshot();
}

void RosOutputAdapter::refresh_execution_times_snapshot() {
  execution_times_snapshot_cache_ = simulator_->get_execution_times_snapshot();
}

custom_interfaces::msg::WheelScalars RosOutputAdapter::to_wheels_msg(
    const common_lib::structures::Wheels& wheels, const rclcpp::Time& stamp) const {
  custom_interfaces::msg::WheelScalars msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "base_link";
  msg.fl = static_cast<float>(wheels.front_left);
  msg.fr = static_cast<float>(wheels.front_right);
  msg.rl = static_cast<float>(wheels.rear_left);
  msg.rr = static_cast<float>(wheels.rear_right);
  return msg;
}

void RosOutputAdapter::publish_tire_group() {
  rclcpp::Time stamp = this->now();

  custom_interfaces::msg::TireForces tire_forces_msg;
  tire_forces_msg.header.stamp = stamp;
  tire_forces_msg.header.frame_id = "base_link";
  tire_forces_msg.fl_force.x = vehicle_model_snapshot_cache_.front_left_force[0];
  tire_forces_msg.fl_force.y = vehicle_model_snapshot_cache_.front_left_force[1];
  tire_forces_msg.fl_force.z = vehicle_model_snapshot_cache_.front_left_force[2];
  tire_forces_msg.fr_force.x = vehicle_model_snapshot_cache_.front_right_force[0];
  tire_forces_msg.fr_force.y = vehicle_model_snapshot_cache_.front_right_force[1];
  tire_forces_msg.fr_force.z = vehicle_model_snapshot_cache_.front_right_force[2];
  tire_forces_msg.rl_force.x = vehicle_model_snapshot_cache_.rear_left_force[0];
  tire_forces_msg.rl_force.y = vehicle_model_snapshot_cache_.rear_left_force[1];
  tire_forces_msg.rl_force.z = vehicle_model_snapshot_cache_.rear_left_force[2];
  tire_forces_msg.rr_force.x = vehicle_model_snapshot_cache_.rear_right_force[0];
  tire_forces_msg.rr_force.y = vehicle_model_snapshot_cache_.rear_right_force[1];
  tire_forces_msg.rr_force.z = vehicle_model_snapshot_cache_.rear_right_force[2];
  tire_forces_pub_->publish(tire_forces_msg);

  tire_slip_ratio_pub_->publish(to_wheels_msg(vehicle_model_snapshot_cache_.slip_ratio, stamp));
  tire_slip_angle_pub_->publish(to_wheels_msg(vehicle_model_snapshot_cache_.slip_angle, stamp));
}

void RosOutputAdapter::publish_motor_group() {
  rclcpp::Time stamp = this->now();

  custom_interfaces::msg::MotorState motor_msg;
  motor_msg.header.stamp = stamp;
  motor_msg.header.frame_id = "base_link";
  motor_msg.torque = vehicle_model_snapshot_cache_.motor_torque;
  motor_msg.omega = vehicle_model_snapshot_cache_.motor_omega;
  motor_msg.rpm = vehicle_model_snapshot_cache_.motor_omega * 60.0 / (2.0 * M_PI);
  motor_msg.current = vehicle_model_snapshot_cache_.motor_current;
  motor_msg.thermal_state = vehicle_model_snapshot_cache_.motor_thermal_state;
  motor_msg.thermal_capacity = vehicle_model_snapshot_cache_.motor_thermal_capacity;
  motor_pub_->publish(motor_msg);
}

void RosOutputAdapter::publish_battery_group() {
  rclcpp::Time stamp = this->now();

  custom_interfaces::msg::BatteryState battery_msg;
  battery_msg.header.stamp = stamp;
  battery_msg.header.frame_id = "base_link";
  battery_msg.voltage = vehicle_model_snapshot_cache_.battery_voltage;
  battery_msg.open_circuit_voltage = vehicle_model_snapshot_cache_.battery_open_circuit_voltage;
  battery_msg.soc = vehicle_model_snapshot_cache_.battery_soc;
  battery_msg.current = vehicle_model_snapshot_cache_.battery_current;
  battery_pub_->publish(battery_msg);
}

void RosOutputAdapter::publish_differential_group() {
  differential_pub_->publish(
      to_wheels_msg(vehicle_model_snapshot_cache_.differential_torque, this->now()));
}

void RosOutputAdapter::publish_aero_group() {
  custom_interfaces::msg::AeroForces aero_msg;
  aero_msg.header.stamp = this->now();
  aero_msg.header.frame_id = "base_link";
  aero_msg.drag = vehicle_model_snapshot_cache_.aero_drag;
  aero_msg.downforce = vehicle_model_snapshot_cache_.aero_downforce;
  aero_pub_->publish(aero_msg);
}

void RosOutputAdapter::publish_load_group() {
  load_pub_->publish(to_wheels_msg(vehicle_model_snapshot_cache_.vertical_load, this->now()));
}

void RosOutputAdapter::publish_status_group() {
  custom_interfaces::msg::VehicleStateVector status_msg;
  status_msg.header.stamp = this->now();
  status_msg.header.frame_id = "base_link";
  status_msg.yaw_rate = vehicle_model_snapshot_cache_.yaw_rate;
  status_msg.velocity_x = vehicle_model_snapshot_cache_.velocity_x;
  status_msg.velocity_y = vehicle_model_snapshot_cache_.velocity_y;
  status_msg.acceleration_x = vehicle_model_snapshot_cache_.acceleration_x;
  status_msg.acceleration_y = vehicle_model_snapshot_cache_.acceleration_y;
  status_msg.steering_angle = vehicle_model_snapshot_cache_.steering_angle;
  auto wheels_speed = vehicle_model_snapshot_cache_.wheel_speed;
  status_msg.fl_rpm = wheels_speed.front_left * 60.0 / (2.0 * M_PI);
  status_msg.fr_rpm = wheels_speed.front_right * 60.0 / (2.0 * M_PI);
  status_msg.rl_rpm = wheels_speed.rear_left * 60.0 / (2.0 * M_PI);
  status_msg.rr_rpm = wheels_speed.rear_right * 60.0 / (2.0 * M_PI);
  status_pub_->publish(status_msg);
}

void RosOutputAdapter::publish_execution_times_group() {
  custom_interfaces::msg::ExecutionTimes times_msg;
  times_msg.header.stamp = this->now();
  times_msg.header.frame_id = "base_link";
  times_msg.powertrain_ms = execution_times_snapshot_cache_.powertrain_ms;
  times_msg.differential_ms = execution_times_snapshot_cache_.differential_ms;
  times_msg.aero_ms = execution_times_snapshot_cache_.aero_ms;
  times_msg.steering_ms = execution_times_snapshot_cache_.steering_ms;
  times_msg.load_transfer_ms = execution_times_snapshot_cache_.load_transfer_ms;
  times_msg.tire_ms = execution_times_snapshot_cache_.tire_ms;
  times_msg.total_step_ms = execution_times_snapshot_cache_.total_step_ms;
  execution_times_pub_->publish(times_msg);
}

void RosOutputAdapter::publish_visualization_group() {
  const rclcpp::Time stamp = this->now();
  const double stamp_sec = stamp.seconds();
  double dt = 0.0;
  if (last_visualization_stamp_sec_ >= 0.0) {
    dt = stamp_sec - last_visualization_stamp_sec_;
  }
  last_visualization_stamp_sec_ = stamp_sec;
  if (dt < 0.0 || dt > 0.2) {
    dt = 0.0;
  }

  visualization_msgs::msg::MarkerArray ground_marker_array;
  visualization_msgs::msg::MarkerArray vehicle_marker_array;

  publish_ground_marker(ground_marker_array, stamp);
  publish_body_marker(vehicle_marker_array, stamp);
  publish_wheel_markers(vehicle_marker_array, stamp, dt);

  visualization_ground_pub_->publish(ground_marker_array);
  visualization_vehicle_pub_->publish(vehicle_marker_array);
}

void RosOutputAdapter::publish_ground_marker(visualization_msgs::msg::MarkerArray& marker_array,
                                             const rclcpp::Time& stamp) const {
  visualization_msgs::msg::Marker ground;
  ground.header.stamp = stamp;
  ground.header.frame_id = "map";
  ground.ns = "invictasim_ground";
  ground.id = 100;
  ground.type = visualization_msgs::msg::Marker::CUBE;
  ground.action = visualization_msgs::msg::Marker::ADD;
  ground.pose.position.x = 0.0;
  ground.pose.position.y = 0.0;
  ground.pose.position.z = -0.02;
  ground.pose.orientation.x = 0.0;
  ground.pose.orientation.y = 0.0;
  ground.pose.orientation.z = 0.0;
  ground.pose.orientation.w = 1.0;
  ground.scale.x = 5000.0;
  ground.scale.y = 5000.0;
  ground.scale.z = 0.04;
  ground.color.a = 0.65f;
  ground.color.r = 0.2f;
  ground.color.g = 0.2f;
  ground.color.b = 0.2f;
  marker_array.markers.push_back(ground);
}

void RosOutputAdapter::publish_body_marker(visualization_msgs::msg::MarkerArray& marker_array,
                                           const rclcpp::Time& stamp) const {
  constexpr double model_offset_x = 0.9;

  tf2::Quaternion q_heading;
  q_heading.setRPY(0.0, 0.0, vehicle_model_snapshot_cache_.yaw);
  tf2::Quaternion q_mesh_offset;
  q_mesh_offset.setRPY(-M_PI_2, 0.0, 0.0);
  tf2::Quaternion q_body = q_heading * q_mesh_offset;
  q_body.normalize();

  visualization_msgs::msg::Marker body;
  body.header.stamp = stamp;
  body.header.frame_id = "map";
  body.ns = "invictasim_vehicle";
  body.id = 0;
  body.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  body.action = visualization_msgs::msg::Marker::ADD;
  body.pose.position.x = vehicle_model_snapshot_cache_.x +
                         std::cos(vehicle_model_snapshot_cache_.yaw) * model_offset_x;
  body.pose.position.y = vehicle_model_snapshot_cache_.y +
                         std::sin(vehicle_model_snapshot_cache_.yaw) * model_offset_x;
  body.pose.position.z = 0.0;
  body.pose.orientation.x = q_body.x();
  body.pose.orientation.y = q_body.y();
  body.pose.orientation.z = q_body.z();
  body.pose.orientation.w = q_body.w();
  body.scale.x = 0.01;
  body.scale.y = 0.01;
  body.scale.z = 0.01;
  body.color.a = 1.0f;
  body.color.r = 0.85f;
  body.color.g = 0.1f;
  body.color.b = 0.1f;
  body.mesh_resource = "package://invictasim/resources/meshes/car_body.stl";
  body.mesh_use_embedded_materials = false;
  marker_array.markers.push_back(body);
}

void RosOutputAdapter::publish_wheel_markers(visualization_msgs::msg::MarkerArray& marker_array,
                                             const rclcpp::Time& stamp, double dt) {
  // PACSIM mesh hardcoded paremeters
  constexpr double model_offset_x = 0.9;
  constexpr double wheel_radius = 0.203;
  constexpr double wheel_center_z = 0.204;

  if (dt > 0.0) {
    const auto wheel_speed = vehicle_model_snapshot_cache_.wheel_speed;
    wheel_spin_fl_ += (wheel_speed.front_left / wheel_radius) * dt;
    wheel_spin_fr_ += (wheel_speed.front_right / wheel_radius) * dt;
    wheel_spin_rl_ += (wheel_speed.rear_left / wheel_radius) * dt;
    wheel_spin_rr_ += (wheel_speed.rear_right / wheel_radius) * dt;
  }

  const double body_x = vehicle_model_snapshot_cache_.x;
  const double body_y = vehicle_model_snapshot_cache_.y;
  const double yaw = vehicle_model_snapshot_cache_.yaw;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double steer = vehicle_model_snapshot_cache_.steering_angle;

  // Hardcoded for current mesh car from pacsim, will be done with parameters with 02
  const double local_x[4] = {
      model_offset_x - 0.0998,
      model_offset_x - 0.0998,
      model_offset_x - 1.64,
      model_offset_x - 1.64,
  };
  const double local_y[4] = {0.6, -0.6, 0.58, -0.58};
  const double steer_angles[4] = {steer, steer, 0.0, 0.0};
  const double spins[4] = {wheel_spin_fl_, wheel_spin_fr_, wheel_spin_rl_, wheel_spin_rr_};

  for (int i = 0; i < 4; ++i) {
    const double world_x = body_x + c * local_x[i] - s * local_y[i];
    const double world_y = body_y + s * local_x[i] + c * local_y[i];

    tf2::Quaternion q_heading;
    q_heading.setRPY(0.0, 0.0, yaw + steer_angles[i]);
    tf2::Quaternion q_spin;
    q_spin.setRPY(0.0, spins[i], 0.0);
    tf2::Quaternion q_mesh_offset;
    q_mesh_offset.setRPY(-M_PI_2, 0.0, 0.0);
    tf2::Quaternion q_wheel = q_heading * q_spin * q_mesh_offset;
    q_wheel.normalize();

    visualization_msgs::msg::Marker wheel;
    wheel.header.stamp = stamp;
    wheel.header.frame_id = "map";
    wheel.ns = "invictasim_vehicle";
    wheel.id = i + 1;
    wheel.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    wheel.action = visualization_msgs::msg::Marker::ADD;
    wheel.pose.position.x = world_x;
    wheel.pose.position.y = world_y;
    wheel.pose.position.z = wheel_center_z;
    wheel.pose.orientation.x = q_wheel.x();
    wheel.pose.orientation.y = q_wheel.y();
    wheel.pose.orientation.z = q_wheel.z();
    wheel.pose.orientation.w = q_wheel.w();
    wheel.scale.x = 0.01;
    wheel.scale.y = 0.01;
    wheel.scale.z = 0.01;
    wheel.color.a = 1.0f;
    wheel.color.r = 0.08f;
    wheel.color.g = 0.08f;
    wheel.color.b = 0.08f;
    wheel.mesh_resource = "package://invictasim/resources/meshes/tire.stl";
    wheel.mesh_use_embedded_materials = false;
    marker_array.markers.push_back(wheel);
  }
}
