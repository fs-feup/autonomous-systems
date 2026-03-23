#include "io/output/ros.hpp"

#include <chrono>
#include <cmath>
#include <set>
#include <thread>

#include "visualization_msgs/msg/marker.hpp"

RosOutputAdapter::RosOutputAdapter(const std::shared_ptr<InvictaSim>& simulator)
    : Node("invictasim_output", rclcpp::NodeOptions().use_global_arguments(false)),
      InvictaSimOutputAdapter(simulator),
      running_(false),
      publish_frequencies_(simulator->get_params().publish_frequencies) {
  tire_forces_pub_ =
      this->create_publisher<custom_interfaces::msg::TireForces>("invictasim/tire/forces", 10);
  tire_slip_ratio_pub_ = this->create_publisher<custom_interfaces::msg::WheelScalars>(
      "invictasim/tire/slip_ratio", 10);
  tire_slip_angle_pub_ = this->create_publisher<custom_interfaces::msg::WheelScalars>(
      "invictasim/tire/slip_angle", 10);
  powertrain_pub_ = this->create_publisher<custom_interfaces::msg::PowertrainState>(
      "invictasim/powertrain/state", 10);
  aero_forces_pub_ =
      this->create_publisher<custom_interfaces::msg::AeroForces>("invictasim/aero/forces", 10);
  wheel_load_pub_ = this->create_publisher<custom_interfaces::msg::WheelScalars>(
      "invictasim/load/vertical_loads", 10);
  vehicle_state_pub_ = this->create_publisher<custom_interfaces::msg::VehicleStateVector>(
      "invictasim/vehicle/state", 10);
  visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
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
  if (publishes_at("tire", frequency_hz)) {
    refresh_tire_snapshot();
    publish_tire_group();
  }
  if (publishes_at("powertrain", frequency_hz)) {
    refresh_powertrain_snapshot();
    publish_powertrain_group();
  }
  if (publishes_at("aero", frequency_hz)) {
    refresh_aero_snapshot();
    publish_aero_group();
  }
  if (publishes_at("load", frequency_hz)) {
    refresh_load_snapshot();
    publish_load_group();
  }
  if (publishes_at("status", frequency_hz)) {
    refresh_status_snapshot();
    publish_status_group();
  }
  if (publishes_at("visualization", frequency_hz)) {
    refresh_status_snapshot();
    publish_visualization_group();
  }
}

void RosOutputAdapter::refresh_tire_snapshot() {
  simulator_->get_tire_snapshot(tire_snapshot_cache_);
}

void RosOutputAdapter::refresh_powertrain_snapshot() {
  simulator_->get_powertrain_snapshot(powertrain_snapshot_cache_);
}

void RosOutputAdapter::refresh_aero_snapshot() {
  simulator_->get_aero_snapshot(aero_snapshot_cache_);
}

void RosOutputAdapter::refresh_load_snapshot() {
  simulator_->get_load_snapshot(load_snapshot_cache_);
}

void RosOutputAdapter::refresh_status_snapshot() {
  simulator_->get_status_snapshot(status_snapshot_cache_);
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
  tire_forces_msg.fl_force.x = tire_snapshot_cache_.front_left_force[0];
  tire_forces_msg.fl_force.y = tire_snapshot_cache_.front_left_force[1];
  tire_forces_msg.fl_force.z = tire_snapshot_cache_.front_left_force[2];
  tire_forces_msg.fr_force.x = tire_snapshot_cache_.front_right_force[0];
  tire_forces_msg.fr_force.y = tire_snapshot_cache_.front_right_force[1];
  tire_forces_msg.fr_force.z = tire_snapshot_cache_.front_right_force[2];
  tire_forces_msg.rl_force.x = tire_snapshot_cache_.rear_left_force[0];
  tire_forces_msg.rl_force.y = tire_snapshot_cache_.rear_left_force[1];
  tire_forces_msg.rl_force.z = tire_snapshot_cache_.rear_left_force[2];
  tire_forces_msg.rr_force.x = tire_snapshot_cache_.rear_right_force[0];
  tire_forces_msg.rr_force.y = tire_snapshot_cache_.rear_right_force[1];
  tire_forces_msg.rr_force.z = tire_snapshot_cache_.rear_right_force[2];
  tire_forces_pub_->publish(tire_forces_msg);

  tire_slip_ratio_pub_->publish(to_wheels_msg(tire_snapshot_cache_.slip_ratio, stamp));
  tire_slip_angle_pub_->publish(to_wheels_msg(tire_snapshot_cache_.slip_angle, stamp));
}

void RosOutputAdapter::publish_powertrain_group() {
  rclcpp::Time stamp = this->now();

  custom_interfaces::msg::PowertrainState powertrain_msg;
  powertrain_msg.header.stamp = stamp;
  powertrain_msg.header.frame_id = "base_link";
  powertrain_msg.motor_torque = powertrain_snapshot_cache_.motor_torque;
  powertrain_msg.motor_omega = powertrain_snapshot_cache_.motor_omega;
  powertrain_msg.motor_rpm = powertrain_snapshot_cache_.motor_omega * 60.0 / (2.0 * M_PI);
  powertrain_msg.motor_current = powertrain_snapshot_cache_.motor_current;
  powertrain_msg.motor_thermal_state = powertrain_snapshot_cache_.motor_thermal_state;
  powertrain_msg.motor_thermal_capacity = powertrain_snapshot_cache_.motor_thermal_capacity;
  powertrain_msg.battery_voltage = powertrain_snapshot_cache_.battery_voltage;
  powertrain_msg.battery_open_circuit_voltage =
      powertrain_snapshot_cache_.battery_open_circuit_voltage;
  powertrain_msg.battery_soc = powertrain_snapshot_cache_.battery_soc;
  powertrain_msg.battery_current = powertrain_snapshot_cache_.battery_current;
  powertrain_msg.diff_torque_fl = powertrain_snapshot_cache_.differential_torque.front_left;
  powertrain_msg.diff_torque_fr = powertrain_snapshot_cache_.differential_torque.front_right;
  powertrain_msg.diff_torque_rl = powertrain_snapshot_cache_.differential_torque.rear_left;
  powertrain_msg.diff_torque_rr = powertrain_snapshot_cache_.differential_torque.rear_right;
  powertrain_pub_->publish(powertrain_msg);
}

void RosOutputAdapter::publish_aero_group() {
  custom_interfaces::msg::AeroForces aero_msg;
  aero_msg.header.stamp = this->now();
  aero_msg.header.frame_id = "base_link";
  aero_msg.drag = aero_snapshot_cache_.drag;
  aero_msg.downforce = aero_snapshot_cache_.downforce;
  aero_forces_pub_->publish(aero_msg);
}

void RosOutputAdapter::publish_load_group() {
  wheel_load_pub_->publish(to_wheels_msg(load_snapshot_cache_.vertical_load, this->now()));
}

void RosOutputAdapter::publish_status_group() {
  custom_interfaces::msg::VehicleStateVector vehicle_state_msg;
  vehicle_state_msg.header.stamp = this->now();
  vehicle_state_msg.header.frame_id = "base_link";
  vehicle_state_msg.velocity_x = status_snapshot_cache_.velocity_x;
  vehicle_state_msg.velocity_y = status_snapshot_cache_.velocity_y;
  vehicle_state_msg.yaw_rate = status_snapshot_cache_.yaw_rate;
  vehicle_state_msg.acceleration_x = status_snapshot_cache_.acceleration_x;
  vehicle_state_msg.acceleration_y = status_snapshot_cache_.acceleration_y;
  vehicle_state_msg.steering_angle = status_snapshot_cache_.steering_angle;
  auto wheels_speed = status_snapshot_cache_.wheel_speed;
  vehicle_state_msg.fl_rpm = wheels_speed.front_left * 60.0 / (2.0 * M_PI);
  vehicle_state_msg.fr_rpm = wheels_speed.front_right * 60.0 / (2.0 * M_PI);
  vehicle_state_msg.rl_rpm = wheels_speed.rear_left * 60.0 / (2.0 * M_PI);
  vehicle_state_msg.rr_rpm = wheels_speed.rear_right * 60.0 / (2.0 * M_PI);
  vehicle_state_pub_->publish(vehicle_state_msg);
}

void RosOutputAdapter::publish_visualization_group() {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = this->now();
  marker.header.frame_id = "map";
  marker.ns = "invictasim_vehicle";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = status_snapshot_cache_.x;
  marker.pose.position.y = status_snapshot_cache_.y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.5;
  marker.scale.y = 1.0;
  marker.scale.z = 0.1;
  marker.color.a = 1.0f;
  marker.color.r = 0.1f;
  marker.color.g = 0.8f;
  marker.color.b = 0.1f;

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(marker);
  visualization_pub_->publish(marker_array);
}
