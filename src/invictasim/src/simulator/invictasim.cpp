#include "simulator/invictasim.hpp"

InvictaSim::InvictaSim(const InvictaSimParameters& params)
    : params_(params),
      running_(false),
      sim_time_(0.0),
      throttle_({0.0, 0.0, 0.0, 0.0}),
      steering_(0.0) {
  // Initialize Objects
  vehicle_model_ = vehicle_models_map.at(params_.vehicle_model.c_str())(params);
  track_ = std::make_shared<Track>(params_.track_name);

  // Set initial position according to track information
  auto start_position = track_->getStartPosition();
  vehicle_model_->set_initial_pose(start_position.x, start_position.y);

  // Initialize step timings
  step_duration_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1 / static_cast<double>(params_.sim_frequency)));
  const auto now = std::chrono::steady_clock::now();
  next_step_time_ = now + step_duration_;
  last_step_time_ = now;
}

void InvictaSim::run() {
  running_ = true;
  while (running_) {
    simulation_step();
  }
}

void InvictaSim::stop() { running_ = false; }

void InvictaSim::simulation_step() {
  auto current_time = std::chrono::steady_clock::now();
  if (current_time < next_step_time_) {
    std::this_thread::sleep_until(next_step_time_);
    current_time = next_step_time_;
  }
  double step_dt = std::chrono::duration<double>(current_time - last_step_time_).count();

  last_step_time_ = current_time;
  next_step_time_ = current_time + step_duration_;
  sim_time_ += step_dt;

  const auto step_start = std::chrono::steady_clock::now();

  // Use a copy of input so no locks are required during vehicle model step.
  const InputSnapshot input_snapshot = get_input_snapshot();

  // Use snapshot throughout step without locks
  vehicle_model_->step(step_dt, input_snapshot.throttle, input_snapshot.steering);

  // Update output snapshot for adapters to read (lock only to copy the data)
  VehicleModelSnapshot vehicle_snapshot = build_vehicle_model_snapshot();
  const auto step_end = std::chrono::steady_clock::now();
  const double total_step_ms =
      std::chrono::duration<double, std::milli>(step_end - step_start).count();
  ExecutionTimesSnapshot execution_times_snapshot = build_execution_times_snapshot(total_step_ms);
  {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    vehicle_model_snapshot_ = vehicle_snapshot;
    execution_times_snapshot_ = execution_times_snapshot;
  }
}

VehicleModelSnapshot InvictaSim::build_vehicle_model_snapshot() const {
  VehicleModelSnapshot snapshot;

  // Tire
  snapshot.front_left_force = vehicle_model_->get_front_left_forces();
  snapshot.front_right_force = vehicle_model_->get_front_right_forces();
  snapshot.rear_left_force = vehicle_model_->get_rear_left_forces();
  snapshot.rear_right_force = vehicle_model_->get_rear_right_forces();
  snapshot.slip_ratio = vehicle_model_->get_wheels_slip_ratio();
  snapshot.slip_angle = vehicle_model_->get_wheels_slip_angle();

  // Motor
  snapshot.motor_torque = vehicle_model_->get_motor_torque();
  snapshot.motor_omega = vehicle_model_->get_motor_omega();
  snapshot.motor_current = vehicle_model_->get_motor_current();
  snapshot.motor_thermal_state = vehicle_model_->get_motor_thermal_state();
  snapshot.motor_thermal_capacity = vehicle_model_->get_motor_thermal_capacity();

  // Battery
  snapshot.battery_current = vehicle_model_->get_battery_current();
  snapshot.battery_voltage = vehicle_model_->get_battery_voltage();
  snapshot.battery_open_circuit_voltage = vehicle_model_->get_battery_open_circuit_voltage();
  snapshot.battery_soc = vehicle_model_->get_battery_soc();

  // Differential
  snapshot.differential_torque = vehicle_model_->get_wheels_torque();

  // Aero
  snapshot.aero_drag = vehicle_model_->get_aero_drag();
  snapshot.aero_downforce = vehicle_model_->get_aero_downforce();

  // Load transfer
  snapshot.vertical_load = vehicle_model_->get_wheels_vertical_load();

  // Status
  snapshot.x = vehicle_model_->get_position_x();
  snapshot.y = vehicle_model_->get_position_y();
  snapshot.yaw = vehicle_model_->get_yaw();
  snapshot.yaw_rate = vehicle_model_->get_yaw_rate();
  snapshot.velocity_x = vehicle_model_->get_velocity_x();
  snapshot.velocity_y = vehicle_model_->get_velocity_y();
  snapshot.acceleration_x = vehicle_model_->get_acceleration_x();
  snapshot.acceleration_y = vehicle_model_->get_acceleration_y();
  snapshot.steering_angle = vehicle_model_->get_steering_angle();
  snapshot.wheel_speed = vehicle_model_->get_wheels_speed();

  return snapshot;
}

ExecutionTimesSnapshot InvictaSim::build_execution_times_snapshot(double total_step_ms) const {
  ExecutionTimesSnapshot snapshot;
  const VehicleModelExecutionTimes model_times = vehicle_model_->get_execution_times();

  snapshot.powertrain_ms = model_times.powertrain_ms;
  snapshot.differential_ms = model_times.differential_ms;
  snapshot.aero_ms = model_times.aero_ms;
  snapshot.steering_ms = model_times.steering_ms;
  snapshot.load_transfer_ms = model_times.load_transfer_ms;
  snapshot.tire_ms = model_times.tire_ms;
  snapshot.total_step_ms = total_step_ms;

  return snapshot;
}
