#include "simulator/invictasim.hpp"

InvictaSim::InvictaSim(const InvictaSimParameters& params)
    : params_(params),
      running_(false),
      sim_time_(0.0),
      throttle_({0.0, 0.0, 0.0, 0.0}),
      steering_(0.0) {
  vehicle_model_ = vehicle_models_map.at(params_.vehicle_model.c_str())(params);
  next_loop_time_ = std::chrono::steady_clock::now();
}

void InvictaSim::run() {
  running_ = true;
  while (running_) {
    simulation_step();
  }
}

void InvictaSim::stop() { running_ = false; }

void InvictaSim::simulation_step() {
  auto now = std::chrono::steady_clock::now();
  if (now < next_loop_time_) {
    std::this_thread::sleep_until(next_loop_time_);
  }
  next_loop_time_ = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(params_.timestep / params_.simulation_speed));

  sim_time_ += params_.timestep;

  // Take copy of input so no locks are required during the vehicle model step
  common_lib::structures::Wheels throttle;
  double steering;
  get_input_snapshot(throttle, steering);

  // Use snapshot throughout step without locks
  vehicle_model_->step(params_.timestep, throttle, steering);

  AggregateOutputSnapshot snapshot;
  snapshot.sim_time = sim_time_;
  snapshot.tire.front_left_force = vehicle_model_->get_front_left_forces();
  snapshot.tire.front_right_force = vehicle_model_->get_front_right_forces();
  snapshot.tire.rear_left_force = vehicle_model_->get_rear_left_forces();
  snapshot.tire.rear_right_force = vehicle_model_->get_rear_right_forces();
  snapshot.tire.slip_ratio = vehicle_model_->get_wheels_slip_ratio();
  snapshot.tire.slip_angle = vehicle_model_->get_wheels_slip_angle();

  snapshot.powertrain.motor_torque = vehicle_model_->get_motor_torque();
  snapshot.powertrain.motor_omega = vehicle_model_->get_motor_omega();
  snapshot.powertrain.motor_current = vehicle_model_->get_motor_current();
  snapshot.powertrain.motor_thermal_state = vehicle_model_->get_motor_thermal_state();
  snapshot.powertrain.motor_thermal_capacity = vehicle_model_->get_motor_thermal_capacity();
  snapshot.powertrain.battery_current = vehicle_model_->get_battery_current();
  snapshot.powertrain.battery_voltage = vehicle_model_->get_battery_voltage();
  snapshot.powertrain.battery_open_circuit_voltage =
      vehicle_model_->get_battery_open_circuit_voltage();
  snapshot.powertrain.battery_soc = vehicle_model_->get_battery_soc();
  snapshot.powertrain.differential_torque = vehicle_model_->get_wheels_torque();

  snapshot.aero.drag = vehicle_model_->get_aero_drag();
  snapshot.aero.downforce = vehicle_model_->get_aero_downforce();

  snapshot.load.vertical_load = vehicle_model_->get_wheels_vertical_load();

  snapshot.status.x = vehicle_model_->get_position_x();
  snapshot.status.y = vehicle_model_->get_position_y();
  snapshot.status.z = vehicle_model_->get_position_z();
  snapshot.status.yaw = vehicle_model_->get_yaw();
  snapshot.status.yaw_rate = vehicle_model_->get_yaw_rate();
  snapshot.status.velocity_x = vehicle_model_->get_velocity_x();
  snapshot.status.velocity_y = vehicle_model_->get_velocity_y();
  snapshot.status.velocity_z = vehicle_model_->get_velocity_z();
  snapshot.status.acceleration_x = vehicle_model_->get_acceleration_x();
  snapshot.status.acceleration_y = vehicle_model_->get_acceleration_y();
  snapshot.status.steering_angle = steering;
  snapshot.status.total_force_x = vehicle_model_->get_total_force_x();
  snapshot.status.total_force_y = vehicle_model_->get_total_force_y();
  snapshot.status.moment_fy = vehicle_model_->get_moment_fy();
  snapshot.status.moment_fx = vehicle_model_->get_moment_fx();
  snapshot.status.self_aligning_moment = vehicle_model_->get_self_aligning_moment();
  snapshot.status.total_torque_z = vehicle_model_->get_total_torque_z();
  snapshot.status.wheel_speed = vehicle_model_->get_wheels_speed();

  {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    latest_output_snapshot_ = snapshot;
  }
}
