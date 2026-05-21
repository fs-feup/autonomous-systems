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
  std::string imu_cfg = common_lib::config_load::get_config_yaml_path(
    "invictasim", "invictasim/sensors", "imu");
  imu_model_ = std::make_unique<IMU>(imu_cfg);
  if (params_.use_simulated_perception) {
    std::string perception_cfg = common_lib::config_load::get_config_yaml_path(
        "invictasim", "invictasim/sensors", "perception");
    perception_model_ = std::make_unique<SimulatedPerception>(perception_cfg);
}

  // Set initial position according to track information
  auto start_position = track_->get_start_position();
  vehicle_model_->set_initial_pose(start_position.x, start_position.y);

  // Initialize step timings
  double initial_sleep_sec = (1.0 / static_cast<double>(params_.sim_frequency)) / params_.sim_speed;
  const auto now = std::chrono::steady_clock::now();
  next_step_time_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(initial_sleep_sec));
  last_step_time_ = now;

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
  params_.sim_speed += delta;
  if (params_.sim_speed < 0.25) {
    params_.sim_speed = 0.25;
  }
}

void InvictaSim::toggle_ebs() {
  std::lock_guard<std::mutex> lock(input_mutex_);
  ebs_active_ = !ebs_active_;
  vehicle_model_->set_ebs(ebs_active_);
}

void InvictaSim::reset_sim() {
  // Send signal to control node to stop sending commands
  go_signal_ = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::lock_guard<std::mutex> lock(input_mutex_);
  // Reset time
  sim_time_ = 0.0;

  // Reset vehicle model
  vehicle_model_->reset();
  auto start_position = track_->get_start_position();
  vehicle_model_->set_initial_pose(start_position.x, start_position.y);

  // Reset EBS
  ebs_active_ = false;
  vehicle_model_->set_ebs(false);

  // Reset inputs
  throttle_ = {0.0, 0.0, 0.0, 0.0};
  steering_ = 0.0;

  // Reset loop timing
  const auto now = std::chrono::steady_clock::now();
  double initial_sleep_sec = (1.0 / static_cast<double>(params_.sim_frequency)) / params_.sim_speed;
  next_step_time_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(initial_sleep_sec));
  last_step_time_ = now;
}

void InvictaSim::simulation_step() {
  auto current_time = std::chrono::steady_clock::now();

  // Calculate step duration
  double step_duration = (1.0 / static_cast<double>(params_.sim_frequency)) / params_.sim_speed;
  auto step_duration_ros = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(step_duration));

  // Sleep until the next step
  if (current_time < next_step_time_) {
    std::this_thread::sleep_until(next_step_time_);
    current_time = std::chrono::steady_clock::now();
  }

  // Real time passed since last step
  double real_dt = std::chrono::duration<double>(current_time - last_step_time_).count();

  // Scale passed time by simulation speed
  double sim_dt = real_dt * params_.sim_speed;

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

  // Compute total step execution time
  const auto step_end = std::chrono::steady_clock::now();
  const double total_step_ms =
      std::chrono::duration<double, std::milli>(step_end - step_start).count();

  // Update output snapshot for adapters to read (lock only to copy the data)
  VehicleModelSnapshot vehicle_snapshot = build_vehicle_model_snapshot();
  ExecutionTimesSnapshot execution_times_snapshot = build_execution_times_snapshot(total_step_ms);
  MapSnapshot map_snapshot = build_map_snapshot(vehicle_snapshot);
  SensorsSnapshot sensors_snapshot = build_sensors_snapshot(vehicle_snapshot);
  VehicleStateSnapshot vehicle_state_snapshot = build_vehicle_state_snapshot();
  {
    std::lock_guard<std::mutex> lock(output_snapshot_mutex_);
    vehicle_model_snapshot_ = vehicle_snapshot;
    execution_times_snapshot_ = execution_times_snapshot;
    map_snapshot_ = map_snapshot;
    sensors_snapshot_ = sensors_snapshot;
    vehicle_state_snapshot_ = vehicle_state_snapshot;
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
  snapshot.total_step_ms = total_step_ms;

  return snapshot;
}

MapSnapshot InvictaSim::build_map_snapshot(const VehicleModelSnapshot& vehicle_snapshot) const {
  MapSnapshot snapshot;
  // For now, all of them publish the same ground truth cones,
  // but later this would publish the slam map and the perception cones
  snapshot.ground_truth = track_->get_cones();
  if (params_.use_simulated_se) {
    snapshot.simulated_slam_map = track_->get_cones();
  } else {
    snapshot.simulated_slam_map = external_slam_cones_;
  }
  if (params_.use_simulated_perception) {
      common_lib::structures::Pose pose;
      pose.position.x = vehicle_snapshot.x;
      pose.position.y = vehicle_snapshot.y;
      pose.orientation = vehicle_snapshot.yaw;
  
      common_lib::structures::Velocities velocities;
      velocities.velocity_x = vehicle_snapshot.velocity_x;
      velocities.velocity_y = vehicle_snapshot.velocity_y;
      velocities.rotational_velocity = vehicle_snapshot.yaw_rate;
    snapshot.perception_cones = perception_model_->perception_error(track_->get_cones(), pose, velocities);
  } else {
    snapshot.perception_cones = external_perception_cones_;
  }
  
  snapshot.perception_exec_time_ms = 0.0;  // Will allow to simualte the perception delay
  snapshot.lap_counter = 0;                // Placeholder for lap counting logic
  return snapshot;
}

SensorsSnapshot InvictaSim::build_sensors_snapshot(
    const VehicleModelSnapshot& vehicle_snapshot) const {
  SensorsSnapshot snapshot;

  auto imu_measurement = imu_model_->apply_imu_error(
      vehicle_snapshot.acceleration_x,
      vehicle_snapshot.acceleration_y,
      vehicle_snapshot.yaw_rate
  );

  snapshot.free_acceleration = Eigen::Vector3d(
      imu_measurement[0],
      imu_measurement[1],
      0.0
  );
  snapshot.angular_velocity = Eigen::Vector3d(
      0.0,
      0.0,
      imu_measurement[2]
  );

  snapshot.wheel_rpm = common_lib::structures::Wheels(
    vehicle_snapshot.wheel_speed.front_left * 60 / (2 * M_PI),
    vehicle_snapshot.wheel_speed.front_right * 60 / (2 * M_PI),
    vehicle_snapshot.wheel_speed.rear_left * 60 / (2 * M_PI),
    vehicle_snapshot.wheel_speed.rear_right * 60 / (2 * M_PI));
  snapshot.steering_angle = vehicle_snapshot.steering_angle;
  snapshot.motor_rpm = vehicle_snapshot.motor_omega * 60 / (2 * M_PI);

  return snapshot;
}

VehicleStateSnapshot InvictaSim::build_vehicle_state_snapshot() const {
  // For now using ground truth values, but later this would incluse state estimation simulation
  // data
  VehicleStateSnapshot snapshot;

  // Pose
  snapshot.position = {vehicle_model_->get_position_x(), vehicle_model_->get_position_y()};
  snapshot.yaw = vehicle_model_->get_yaw();
  snapshot.pose_covariance = std::vector<double>(9, 0.0);  // Placeholder for pose covariance

  // Velocities
  snapshot.velocity_x = vehicle_model_->get_velocity_x();
  snapshot.velocity_y = vehicle_model_->get_velocity_y();
  snapshot.yaw_rate = vehicle_model_->get_yaw_rate();
  snapshot.velocity_covariance =
      std::vector<double>(9, 0.0);  // Placeholder for velocity covariance

  // Operational status
  snapshot.go_signal = go_signal_;
  snapshot.mission = common_lib::competition_logic::get_mission_from_string(params_.discipline);
  return snapshot;
}
