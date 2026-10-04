#include "simulator/invictasim.hpp"

#include <algorithm>
#include <iostream>
#include <thread>

#include "common_lib/config_load/config_load.hpp"
#include "config/resource_paths.hpp"

InvictaSim::InvictaSim(const InvictaSimParameters& params)
    : params_(params),
      running_(false),
      sim_speed_(params.sim_speed),
      reset_pending_(false),
      sim_time_(0.0),
      throttle_({0.0, 0.0, 0.0, 0.0}),
      steering_(0.0) {
  // Initialize Objects
  vehicle_model_ = vehicle_models_map.at(params_.vehicle_model.c_str())(params);
  track_ = std::make_shared<Track>(params_.track_name);
  statistics_ =
      std::make_unique<Statistics>(*track_, params_.discipline, params_.car_parameters_config);
  initialize_sensors();

  // Set initial position according to track information, allowing for this car's length
  const auto start_position = track_->get_start_position(
      invictasim::resources::car_front_offset(params_.car_parameters_config));
  vehicle_model_->set_initial_pose(start_position.x, start_position.y);

  // Initialize step timings
  double initial_sleep_sec = (1.0 / static_cast<double>(params_.sim_frequency)) / sim_speed_;
  const auto now = std::chrono::steady_clock::now();
  next_step_time_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(initial_sleep_sec));
  last_step_time_ = now;
  last_stall_report_ = now;

  // Initialize go_signal from config
  go_signal_ = params_.initial_go_signal;
}

void InvictaSim::run() {
  running_ = true;
  while (running_) {
    simulation_step();
  }
}

void InvictaSim::stop() { running_ = false; }

void InvictaSim::activate_go_signal() { go_signal_ = true; }

void InvictaSim::add_sim_speed(double delta) {
  sim_speed_ = std::clamp(sim_speed_ + delta, 0.25, 10.0);
}

void InvictaSim::initialize_sensors() {
  const auto sensor_config = [](const std::string& name) {
    return common_lib::config_load::get_config_yaml_path("invictasim", "invictasim/sensors", name);
  };

  // The IMU and wheel speed sensors always stand in for real hardware, so they are always
  // simulated. Perception is only simulated when the real perception node is not running.
  imu_ = std::make_unique<IMU>(sensor_config("imu"));
  wss_ = std::make_unique<WSS>(sensor_config("wss"));
  if (params_.use_simulated_perception) {
    perception_ = std::make_unique<SimulatedPerception>(sensor_config("perception"));
  }
}

void InvictaSim::toggle_ebs() {
  std::lock_guard<std::mutex> lock(input_mutex_);
  ebs_active_ = !ebs_active_;
  vehicle_model_->set_ebs(ebs_active_);
}

void InvictaSim::reset_sim() {
  // Drop the go signal first so the control node stops commanding while the car is moved, then
  // hand the reset to the simulation thread. Touching the vehicle model from this thread would
  // race the step that is reading it.
  go_signal_ = false;
  reset_pending_ = true;

  // Give the simulation thread a chance to pick the request up before returning, so the hotkey
  // feedback the caller prints is not ahead of the reset.
  for (int attempt = 0; attempt < 100 && reset_pending_ && running_; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void InvictaSim::apply_reset() {
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    throttle_ = {0.0, 0.0, 0.0, 0.0};
    steering_ = 0.0;
    ebs_active_ = false;
  }

  sim_time_ = 0.0;

  vehicle_model_->reset();
  vehicle_model_->set_ebs(false);
  statistics_->reset();
  if (perception_) {
    perception_->reset();
  }

  const auto start_position = track_->get_start_position(
      invictasim::resources::car_front_offset(params_.car_parameters_config));
  vehicle_model_->set_initial_pose(start_position.x, start_position.y);

  go_signal_ = params_.initial_go_signal;

  const auto now = std::chrono::steady_clock::now();
  const double initial_sleep_sec = (1.0 / static_cast<double>(params_.sim_frequency)) / sim_speed_;
  next_step_time_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(initial_sleep_sec));
  last_step_time_ = now;
}

void InvictaSim::simulation_step() {
  if (reset_pending_.exchange(false)) {
    apply_reset();
  }

  const double sim_speed = sim_speed_;
  auto current_time = std::chrono::steady_clock::now();

  // Calculate step duration
  const double step_duration = (1.0 / static_cast<double>(params_.sim_frequency)) / sim_speed;
  auto step_duration_ros = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(step_duration));

  // Sleep until the next step
  if (current_time < next_step_time_) {
    std::this_thread::sleep_until(next_step_time_);
    current_time = std::chrono::steady_clock::now();
  }

  // Real time passed since last step
  const double real_dt = std::chrono::duration<double>(current_time - last_step_time_).count();

  // Scale passed time by simulation speed, and cap the step. A stalled thread would otherwise ask
  // the vehicle model to advance further than its substepping can integrate stably, which shows up
  // as runaway wheel spin. Capping makes the simulation briefly fall behind real time instead.
  const double max_sim_dt = 10.0 * step_duration * sim_speed;
  const double sim_dt = std::min(real_dt * sim_speed, max_sim_dt);
  if (real_dt * sim_speed > max_sim_dt) {
    // Rate-limited: the first few steps after startup and after a reset routinely overrun while
    // the process warms up, and a message per step would bury anything else.
    ++stalled_step_count_;
    const auto now_wall = std::chrono::steady_clock::now();
    if (now_wall - last_stall_report_ > std::chrono::seconds(5)) {
      std::cerr << "[InvictaSim] " << stalled_step_count_
                << " simulation step(s) overran since the last report; the most recent took "
                << static_cast<int>(real_dt * 1000.0) << " ms and was capped at "
                << static_cast<int>(max_sim_dt * 1000.0) << " ms." << std::endl;
      last_stall_report_ = now_wall;
      stalled_step_count_ = 0;
    }
  }

  // Update next step
  last_step_time_ = current_time;
  next_step_time_ = current_time + step_duration_ros;

  // Increase simulation time
  sim_time_ += sim_dt;
  const auto step_start = std::chrono::steady_clock::now();

  // Use a copy of input so no locks are required during vehicle model step.
  const InputSnapshot input_snapshot = get_input_snapshot();

  // Use snapshot throughout step without locks
  vehicle_model_->step(sim_dt, input_snapshot.throttle, input_snapshot.steering);
  VehicleModelSnapshot vehicle_snapshot = build_vehicle_model_snapshot();

  static const std::vector<common_lib::structures::PathPoint> kNoPathPoints;
  statistics_->update(
      vehicle_snapshot, sim_time_, sim_dt,
      input_snapshot.external_path_points ? *input_snapshot.external_path_points : kNoPathPoints);

  // Compute total step execution time
  const auto step_end = std::chrono::steady_clock::now();
  const double total_step_ms =
      std::chrono::duration<double, std::milli>(step_end - step_start).count();

  // Update output snapshot for adapters to read (lock only to copy the data)
  ExecutionTimesSnapshot execution_times_snapshot = build_execution_times_snapshot(total_step_ms);
  StatisticsSnapshot statistics_snapshot = statistics_->get_snapshot();
  MapSnapshot map_snapshot = build_map_snapshot(input_snapshot, sim_dt);
  SensorsSnapshot sensors_snapshot = build_sensors_snapshot(vehicle_snapshot);
  VehicleStateSnapshot vehicle_state_snapshot = build_vehicle_state_snapshot();
  {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    vehicle_model_snapshot_ = vehicle_snapshot;
    execution_times_snapshot_ = execution_times_snapshot;
    map_snapshot_ = map_snapshot;
    sensors_snapshot_ = sensors_snapshot;
    vehicle_state_snapshot_ = vehicle_state_snapshot;
    statistics_snapshot_ = statistics_snapshot;
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

  // Transmission
  snapshot.transmission_torque = vehicle_model_->get_wheels_torque();

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
  snapshot.transmission_ms = model_times.transmission_ms;
  snapshot.aero_ms = model_times.aero_ms;
  snapshot.steering_ms = model_times.steering_ms;
  snapshot.load_transfer_ms = model_times.load_transfer_ms;
  snapshot.tire_ms = model_times.tire_ms;
  snapshot.integration_ms = model_times.integration_ms;
  snapshot.total_step_ms = total_step_ms;

  return snapshot;
}

MapSnapshot InvictaSim::build_map_snapshot(const InputSnapshot& input_snapshot, double sim_dt) {
  MapSnapshot snapshot;

  snapshot.ground_truth = track_->get_cones();
  snapshot.cones_hit_this_lap = statistics_->get_cones_hit_this_lap();

  // The simulated SLAM map stands in for a completed mapping run, so it is the ground truth in the
  // map frame. Perception, in contrast, reports what the sensor can currently see, in the vehicle
  // frame, which is what the simulated LiDAR model produces.
  if (params_.use_simulated_se) {
    snapshot.simulated_slam_map = track_->get_cones();
  } else {
    if (input_snapshot.external_slam_cones) {
      snapshot.simulated_slam_map = *input_snapshot.external_slam_cones;
    }
  }

  if (perception_) {
    const auto start = std::chrono::steady_clock::now();
    common_lib::structures::Pose pose;
    pose.position = {vehicle_model_->get_position_x(), vehicle_model_->get_position_y()};
    pose.orientation = vehicle_model_->get_yaw();
    snapshot.perception_cones = perception_->perception_error(snapshot.ground_truth, pose, sim_dt);
    snapshot.perception_exec_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  } else {
    if (input_snapshot.external_perception_cones) {
      snapshot.perception_cones = *input_snapshot.external_perception_cones;
    }
    snapshot.perception_exec_time_ms = 0.0;
  }

  return snapshot;
}

SensorsSnapshot InvictaSim::build_sensors_snapshot(const VehicleModelSnapshot& vehicle_snapshot) {
  SensorsSnapshot snapshot;

  // These are the sensor channels the car reads on the real vehicle, so they carry the configured
  // bias, scale and noise rather than the model's own state.
  const ImuMeasurement imu = imu_->apply_imu_error(
      vehicle_snapshot.acceleration_x, vehicle_snapshot.acceleration_y, vehicle_snapshot.yaw_rate);
  snapshot.free_acceleration = Eigen::Vector3d(imu.acceleration_x, imu.acceleration_y, 0.0);
  snapshot.angular_velocity = Eigen::Vector3d(0.0, 0.0, imu.yaw_rate);

  snapshot.wheel_rpm = wss_->simulate_wheel_speeds(vehicle_snapshot.wheel_speed);
  snapshot.wheel_dropout = wss_->get_wheel_dropout_status();

  snapshot.steering_angle = vehicle_snapshot.steering_angle;
  snapshot.motor_rpm = vehicle_snapshot.motor_omega * 60.0 / (2.0 * M_PI);
  return snapshot;
}

VehicleStateSnapshot InvictaSim::build_vehicle_state_snapshot() const {
  // The simulated state estimator is perfect: it reports the model's own state, with zero
  // covariance, so downstream nodes can be exercised without a state estimation node running.
  VehicleStateSnapshot snapshot;

  // Pose
  snapshot.position = {vehicle_model_->get_position_x(), vehicle_model_->get_position_y()};
  snapshot.yaw = vehicle_model_->get_yaw();
  snapshot.pose_covariance = std::vector<double>(9, 0.0);  // Placeholder for pose covariance

  // Velocities
  snapshot.velocity_x = vehicle_model_->get_velocity_x();
  snapshot.velocity_y = vehicle_model_->get_velocity_y();
  snapshot.yaw_rate = vehicle_model_->get_yaw_rate();
  snapshot.acceleration_x = vehicle_model_->get_acceleration_x();
  snapshot.acceleration_y = vehicle_model_->get_acceleration_y();
  snapshot.steering_angle = vehicle_model_->get_steering_angle();
  const auto wheel_speed = vehicle_model_->get_wheels_speed();
  snapshot.wheel_rpm = common_lib::structures::Wheels(
      wheel_speed.front_left * 60.0 / (2.0 * M_PI), wheel_speed.front_right * 60.0 / (2.0 * M_PI),
      wheel_speed.rear_left * 60.0 / (2.0 * M_PI), wheel_speed.rear_right * 60.0 / (2.0 * M_PI));
  snapshot.velocity_covariance =
      std::vector<double>(9, 0.0);  // Placeholder for velocity covariance

  // Operational status
  snapshot.go_signal = go_signal_;
  snapshot.mission = common_lib::competition_logic::get_mission_from_string(params_.discipline);
  return snapshot;
}
