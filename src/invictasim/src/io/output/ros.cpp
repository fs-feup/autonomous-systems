#include "io/output/ros.hpp"

RosOutputAdapter::RosOutputAdapter(const std::shared_ptr<InvictaSim>& simulator,
                                   const std::string& config_file)
    : Node("invictasim_output", rclcpp::NodeOptions().use_global_arguments(false)),
      InvictaSimOutputAdapter(simulator),
      running_(true) {
  // Vehicle model publishers
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
  transmission_pub_ = this->create_publisher<custom_interfaces::msg::WheelScalars>(
      "invictasim/vehicle_model/transmission", 10);
  aero_pub_ = this->create_publisher<custom_interfaces::msg::AeroForces>(
      "invictasim/vehicle_model/aero", 10);
  status_pub_ = this->create_publisher<custom_interfaces::msg::VehicleStateVector>(
      "invictasim/vehicle_model/status", 10);

  // Input + execution times + map publishers
  input_command_pub_ =
      this->create_publisher<custom_interfaces::msg::ControlCommand>("invictasim/input", 10);
  execution_times_pub_ = this->create_publisher<custom_interfaces::msg::ExecutionTimes>(
      "invictasim/execution_times", 10);
  map_pub_ = this->create_publisher<custom_interfaces::msg::ConeArray>("invictasim/map", 10);

  // Statistics publishers
  lap_summary_pub_ = this->create_publisher<custom_interfaces::msg::LapSummary>(
      "invictasim/statistics/lap_summary", rclcpp::QoS(10).transient_local());
  lap_current_pub_ = this->create_publisher<custom_interfaces::msg::LapCurrent>(
      "invictasim/statistics/lap_current", 10);
  control_statistics_pub_ = this->create_publisher<custom_interfaces::msg::ControlStatistics>(
      "invictasim/statistics/control_statistics", 10);

  // Visualization Publishers
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  visualization_ground_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "invictasim/visualization/ground", 10);
  visualization_vehicle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "invictasim/visualization/vehicle", 10);
  visualization_gt_cones_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "invictasim/visualization/ground_truth_cones", 10);
  visualization_slam_cones_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "invictasim/visualization/slam_cones", 10);
  visualization_perception_cones_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "invictasim/visualization/perception_cones", 10);

  // Simulated perception publishers
  if (simulator_->get_params().use_simulated_perception) {
    perception_pub_ = this->create_publisher<custom_interfaces::msg::PerceptionOutput>(
        "invictasim/perception/cones", 10);
  }

  // Simulated state estimation publishers
  if (simulator_->get_params().use_simulated_se) {
    state_map_pub_ = this->create_publisher<custom_interfaces::msg::ConeArray>(
        "invictasim/state_estimation/map", 10);
    vehicle_pose_pub_ = this->create_publisher<custom_interfaces::msg::Pose>(
        "invictasim/state_estimation/vehicle_pose", 10);
    vehicle_state_vector_pub_ = this->create_publisher<custom_interfaces::msg::VehicleStateVector>(
        "invictasim/state_estimation/vehicle_state_vector", 10);
    lap_counter_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "invictasim/state_estimation/lap_counter", 10);
  }

  // Simulated velocities publisher
  if (simulator_->get_params().use_simulated_velocities) {
    velocities_pub_ = this->create_publisher<custom_interfaces::msg::Velocities>(
        "invictasim/state_estimation/velocities", 10);
  }

  // Sensors
  free_accel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "invictasim/imu/free_acceleration", 10);
  angular_vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "invictasim/imu/angular_velocity", 10);
  vehicle_fl_rpm_pub_ =
      this->create_publisher<custom_interfaces::msg::WheelRPM>("invictasim/wss/front_left", 10);
  vehicle_fr_rpm_pub_ =
      this->create_publisher<custom_interfaces::msg::WheelRPM>("invictasim/wss/front_right", 10);
  vehicle_motor_rpm_pub_ =
      this->create_publisher<custom_interfaces::msg::WheelRPM>("invictasim/resolver", 10);
  steering_pub_ = this->create_publisher<custom_interfaces::msg::SteeringAngle>(
      "invictasim/steering_angle_sensor", 10);

  // Operational status
  operational_status_pub_ = this->create_publisher<custom_interfaces::msg::OperationalStatus>(
      "invictasim/operational_status", 10);

  // Load configs and setup timers
  load_visualization_resources();
  load_publish_frequencies(config_file);
  setup_timers();
}

void RosOutputAdapter::run() {}

void RosOutputAdapter::stop() { running_ = false; }

void RosOutputAdapter::load_publish_frequencies(const std::string& config_file) {
  std::string full_path =
      common_lib::config_load::get_config_yaml_path("invictasim", "invictasim/output", config_file);
  YAML::Node config = YAML::LoadFile(full_path);
  if (!config["publish_frequencies"]) return;

  load_group_from_yaml(config, "vehicle_model");
  load_group_from_yaml(config, "visualization");
  load_group_from_yaml(config, "sensors");
  load_group_from_yaml(config, "map");
  load_group_from_yaml(config, "vehicle_state");
  load_group_from_yaml(config, "statistics");

  if (config["publish_frequencies"]["execution_time"]) {
    topic_frequencies_["execution_time"] =
        config["publish_frequencies"]["execution_time"].as<int>();
  }

  // After loading the config, build the dispatch table
  map_callbacks();
}

void RosOutputAdapter::map_callbacks() {
  const auto no_refresh = []() {};
  const auto refresh_vehicle_model = [this]() { refresh_vehicle_model_snapshot(); };
  const auto refresh_execution_times = [this]() { refresh_execution_times_snapshot(); };
  const auto refresh_map = [this]() { refresh_map_snapshot(); };
  const auto refresh_sensors = [this]() { refresh_sensors_snapshot(); };
  const auto refresh_vehicle_state = [this]() { refresh_vehicle_state_snapshot(); };
  const auto refresh_statistics = [this]() { refresh_statistics_snapshot(); };

  // Vehicle model
  register_pub_helper("tire", refresh_vehicle_model,
                      [this](const rclcpp::Time& stamp) { publish_vm_tire(stamp); });
  register_pub_helper("motor", refresh_vehicle_model,
                      [this](const rclcpp::Time& stamp) { publish_vm_motor(stamp); });
  register_pub_helper("battery", refresh_vehicle_model,
                      [this](const rclcpp::Time& stamp) { publish_vm_battery(stamp); });
  register_pub_helper("transmission", refresh_vehicle_model,
                      [this](const rclcpp::Time& stamp) { publish_vm_transmission(stamp); });
  register_pub_helper("aero", refresh_vehicle_model,
                      [this](const rclcpp::Time& stamp) { publish_vm_aero(stamp); });
  register_pub_helper("status", refresh_vehicle_model, [this](const rclcpp::Time& stamp) {
    publish_vm_status(stamp);
    publish_input(stamp);
  });

  // Visualization
  register_pub_helper("car", refresh_vehicle_model,
                      [this](const rclcpp::Time& stamp) { publish_visualization_car(stamp); });
  register_pub_helper("ground", no_refresh,
                      [this](const rclcpp::Time& stamp) { publish_visualization_ground(stamp); });
  register_pub_helper("ground_truth_cones", refresh_map,
                      [this](const rclcpp::Time& stamp) { publish_visualization_gt_cones(stamp); });

  // Sensors
  register_pub_helper("imu", refresh_sensors,
                      [this](const rclcpp::Time& stamp) { publish_sensors_imu(stamp); });
  register_pub_helper("wheel_speed", refresh_sensors,
                      [this](const rclcpp::Time& stamp) { publish_sensors_wheel_speed(stamp); });
  register_pub_helper("resolver", refresh_sensors,
                      [this](const rclcpp::Time& stamp) { publish_sensors_resolver(stamp); });
  register_pub_helper("steering", refresh_sensors,
                      [this](const rclcpp::Time& stamp) { publish_sensors_steering(stamp); });

  // Map
  register_pub_helper("ground_truth", refresh_map,
                      [this](const rclcpp::Time& stamp) { publish_map_ground_truth(stamp); });

  // Operational status
  register_pub_helper("operational_status", refresh_vehicle_state,
                      [this](const rclcpp::Time& stamp) { publish_operational_status(stamp); });

  // Exec times
  register_pub_helper("execution_time", refresh_execution_times,
                      [this](const rclcpp::Time& stamp) { publish_execution_time(stamp); });

  // Statistics
  register_pub_helper("lap_summary", refresh_statistics,
                      [this](const rclcpp::Time& stamp) { publish_lap_summary(stamp); });
  register_pub_helper("lap_current", refresh_statistics,
                      [this](const rclcpp::Time& stamp) { publish_lap_current(stamp); });
  register_pub_helper("control_statistics", refresh_statistics,
                      [this](const rclcpp::Time& stamp) { publish_control_statistics(stamp); });

  // SLAM Cones Visualization (either external or simulated)
  register_pub_helper("slam_cones", refresh_map, [this](const rclcpp::Time& stamp) {
    publish_visualization_slam_cones(stamp);
  });

  // Perception Cones Visualization (either external or simulated)
  register_pub_helper("perception_cones", refresh_map, [this](const rclcpp::Time& stamp) {
    publish_visualization_perception_cones(stamp);
  });

  // Simulated state estimation
  if (simulator_->get_params().use_simulated_se) {
    register_pub_helper("simulated_slam", refresh_map,
                        [this](const rclcpp::Time& stamp) { publish_state_estimation_map(stamp); });
    register_pub_helper("lap_counter", refresh_statistics,
                        [this](const rclcpp::Time&) { publish_state_estimation_lap_counter(); });
    register_pub_helper("pose", refresh_vehicle_state,
                        [this](const rclcpp::Time& stamp) { publish_state_estimation_pose(stamp); });
    register_pub_helper("state_vector", refresh_vehicle_state, [this](const rclcpp::Time& stamp) {
      publish_state_estimation_state_vector(stamp);
    });
  }

  // Simulated perception
  if (simulator_->get_params().use_simulated_perception) {
    register_pub_helper("perception", refresh_map,
                        [this](const rclcpp::Time& stamp) { publish_perception_cones(stamp); });
  }

  // Simulated velocities
  if (simulator_->get_params().use_simulated_velocities) {
    register_pub_helper("velocities", refresh_vehicle_state, [this](const rclcpp::Time& stamp) {
      publish_state_estimation_velocities(stamp);
    });
  }
}

void RosOutputAdapter::load_group_from_yaml(const YAML::Node& config,
                                            const std::string& group_name) {
  const YAML::Node& group_node = config["publish_frequencies"][group_name];
  if (group_node && group_node.IsMap()) {
    for (const auto& node : group_node) {
      std::string topic_key = node.first.as<std::string>();
      int frequency = node.second.as<int>();

      topic_frequencies_[topic_key] = frequency;
    }
  }
}

void RosOutputAdapter::register_pub_helper(const std::string& topic,
                                           std::function<void()> refresh_snapshot,
                                           std::function<void(const rclcpp::Time&)> func) {
  if (topic_frequencies_.count(topic) && topic_frequencies_[topic] > 0) {
    const int frequency = topic_frequencies_[topic];
    frequency_callbacks_[frequency].push_back(
        [refresh_snapshot, func](const rclcpp::Time& stamp) {
          refresh_snapshot();
          func(stamp);
        });
  }
}

void RosOutputAdapter::setup_timers() {
  for (const auto& [frequency_hz, callbacks] : frequency_callbacks_) {
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(frequency_hz)));

    frequency_timers_[frequency_hz] = this->create_wall_timer(period, [this, frequency_hz]() {
      if (!running_) return;
      on_frequency_tick(frequency_hz);
    });
  }
}

void RosOutputAdapter::load_visualization_resources() {
  load_cone_visualization_resources();
  load_ground_visualization_resources();
  load_car_visualization_resources();
}

void RosOutputAdapter::on_frequency_tick(int frequency_hz) {
  const rclcpp::Time stamp = this->now();

  // Execute functions for this frequency
  for (const auto& publish_func : frequency_callbacks_[frequency_hz]) {
    publish_func(stamp);
  }
}

void RosOutputAdapter::refresh_vehicle_model_snapshot() {
  vehicle_model_snapshot_cache_ = simulator_->get_vehicle_model_snapshot();
}

void RosOutputAdapter::refresh_execution_times_snapshot() {
  execution_times_snapshot_cache_ = simulator_->get_execution_times_snapshot();
}

void RosOutputAdapter::refresh_map_snapshot() {
  map_snapshot_cache_ = simulator_->get_map_snapshot();
}

void RosOutputAdapter::refresh_sensors_snapshot() {
  sensors_snapshot_cache_ = simulator_->get_sensors_snapshot();
}

void RosOutputAdapter::refresh_vehicle_state_snapshot() {
  vehicle_state_snapshot_cache_ = simulator_->get_vehicle_state_snapshot();
}

void RosOutputAdapter::refresh_statistics_snapshot() {
  statistics_snapshot_cache_ = simulator_->get_statistics_snapshot();
}

void RosOutputAdapter::publish_sensors_imu(const rclcpp::Time& stamp) {
  geometry_msgs::msg::Vector3Stamped free_accel_msg;
  free_accel_msg.header.stamp = stamp;
  free_accel_msg.header.frame_id = "base_link";
  free_accel_msg.vector.x = sensors_snapshot_cache_.free_acceleration.x();
  free_accel_msg.vector.y = sensors_snapshot_cache_.free_acceleration.y();
  free_accel_msg.vector.z = sensors_snapshot_cache_.free_acceleration.z();
  free_accel_pub_->publish(free_accel_msg);

  geometry_msgs::msg::Vector3Stamped angular_vel_msg;
  angular_vel_msg.header.stamp = stamp;
  angular_vel_msg.header.frame_id = "base_link";
  angular_vel_msg.vector.x = sensors_snapshot_cache_.angular_velocity.x();
  angular_vel_msg.vector.y = sensors_snapshot_cache_.angular_velocity.y();
  angular_vel_msg.vector.z = sensors_snapshot_cache_.angular_velocity.z();
  angular_vel_pub_->publish(angular_vel_msg);
}

void RosOutputAdapter::publish_sensors_wheel_speed(const rclcpp::Time& stamp) {
  // Publish wheel RPMs for each wheel
  custom_interfaces::msg::WheelRPM fl_msg;
  fl_msg.header.stamp = stamp;
  fl_msg.header.frame_id = "base_link";
  fl_msg.fl_rpm = sensors_snapshot_cache_.wheel_rpm.front_left;
  vehicle_fl_rpm_pub_->publish(fl_msg);

  custom_interfaces::msg::WheelRPM fr_msg = fl_msg;
  fr_msg.fr_rpm = sensors_snapshot_cache_.wheel_rpm.front_right;
  vehicle_fr_rpm_pub_->publish(fr_msg);
}

void RosOutputAdapter::publish_sensors_resolver(const rclcpp::Time& stamp) {
  custom_interfaces::msg::WheelRPM resolver_msg;
  resolver_msg.header.stamp = stamp;
  resolver_msg.header.frame_id = "base_link";
  resolver_msg.rr_rpm =
      sensors_snapshot_cache_.motor_rpm;  // Using rr_rpm field to publish motor rpm
  vehicle_motor_rpm_pub_->publish(resolver_msg);
}

void RosOutputAdapter::publish_sensors_steering(const rclcpp::Time& stamp) {
  custom_interfaces::msg::SteeringAngle steering_msg;
  steering_msg.header.stamp = stamp;
  steering_msg.header.frame_id = "base_link";
  steering_msg.steering_angle = sensors_snapshot_cache_.steering_angle;
  steering_pub_->publish(steering_msg);
}

void RosOutputAdapter::publish_map_ground_truth(const rclcpp::Time& stamp) {
  const auto cones = mark_recently_hit_cones_red(map_snapshot_cache_.ground_truth);
  custom_interfaces::msg::ConeArray track_msg;
  track_msg.header.stamp = stamp;
  track_msg.header.frame_id = "map";
  track_msg.cone_array.reserve(cones.size());

  for (const auto& cone : cones) {
    custom_interfaces::msg::Cone msg;
    msg.position.x = cone.position.x;
    msg.position.y = cone.position.y;
    msg.color = common_lib::competition_logic::get_color_string(cone.color);
    msg.confidence = cone.certainty;
    msg.is_large = cone.is_large;
    track_msg.cone_array.push_back(msg);
  }

  map_pub_->publish(track_msg);
}

void RosOutputAdapter::publish_state_estimation_map(const rclcpp::Time& stamp) {
  const auto cones = mark_recently_hit_cones_red(map_snapshot_cache_.simulated_slam_map);
  custom_interfaces::msg::ConeArray map_msg;
  map_msg.header.stamp = stamp;
  map_msg.header.frame_id = "map";
  map_msg.cone_array.reserve(cones.size());

  for (const auto& cone : cones) {
    custom_interfaces::msg::Cone msg;
    msg.position.x = cone.position.x;
    msg.position.y = cone.position.y;
    msg.color = common_lib::competition_logic::get_color_string(cone.color);
    msg.confidence = cone.certainty;
    msg.is_large = cone.is_large;
    map_msg.cone_array.push_back(msg);
  }

  state_map_pub_->publish(map_msg);
}

void RosOutputAdapter::publish_state_estimation_lap_counter() {
  std_msgs::msg::Float64 lap_msg;
  lap_msg.data = static_cast<double>(statistics_snapshot_cache_.lap_counter);
  lap_counter_pub_->publish(lap_msg);
}

void RosOutputAdapter::publish_perception_cones(const rclcpp::Time& stamp) {
  const auto& cones = map_snapshot_cache_.perception_cones;
  custom_interfaces::msg::PerceptionOutput perception_msg;
  perception_msg.header.stamp = stamp;
  perception_msg.header.frame_id = "base_link";
  perception_msg.cones.header = perception_msg.header;
  perception_msg.cones.cone_array.reserve(cones.size());
  for (const auto& cone : cones) {
    custom_interfaces::msg::Cone msg;
    msg.position.x = cone.position.x;
    msg.position.y = cone.position.y;
    msg.color = common_lib::competition_logic::get_color_string(cone.color);
    msg.confidence = cone.certainty;
    msg.is_large = cone.is_large;
    perception_msg.cones.cone_array.push_back(msg);
  }
  perception_msg.exec_time = map_snapshot_cache_.perception_exec_time_ms;
  perception_pub_->publish(perception_msg);
}

void RosOutputAdapter::publish_vm_tire(const rclcpp::Time& stamp) {
  custom_interfaces::msg::TireForces tire_forces_msg;
  tire_forces_msg.header.stamp = stamp;
  tire_forces_msg.header.frame_id = "base_link";

  // Front Left
  tire_forces_msg.fl_wrench.force.x = vehicle_model_snapshot_cache_.front_left_force[0];
  tire_forces_msg.fl_wrench.force.y = vehicle_model_snapshot_cache_.front_left_force[1];
  tire_forces_msg.fl_wrench.force.z = vehicle_model_snapshot_cache_.vertical_load.front_left;
  tire_forces_msg.fl_wrench.torque.y = vehicle_model_snapshot_cache_.front_left_force[2];
  tire_forces_msg.fl_wrench.torque.z = vehicle_model_snapshot_cache_.front_left_force[3];

  // Front Right
  tire_forces_msg.fr_wrench.force.x = vehicle_model_snapshot_cache_.front_right_force[0];
  tire_forces_msg.fr_wrench.force.y = vehicle_model_snapshot_cache_.front_right_force[1];
  tire_forces_msg.fr_wrench.force.z = vehicle_model_snapshot_cache_.vertical_load.front_right;
  tire_forces_msg.fr_wrench.torque.y = vehicle_model_snapshot_cache_.front_right_force[2];
  tire_forces_msg.fr_wrench.torque.z = vehicle_model_snapshot_cache_.front_right_force[3];

  // Rear Left
  tire_forces_msg.rl_wrench.force.x = vehicle_model_snapshot_cache_.rear_left_force[0];
  tire_forces_msg.rl_wrench.force.y = vehicle_model_snapshot_cache_.rear_left_force[1];
  tire_forces_msg.rl_wrench.force.z = vehicle_model_snapshot_cache_.vertical_load.rear_left;
  tire_forces_msg.rl_wrench.torque.y = vehicle_model_snapshot_cache_.rear_left_force[2];
  tire_forces_msg.rl_wrench.torque.z = vehicle_model_snapshot_cache_.rear_left_force[3];

  // Rear Right
  tire_forces_msg.rr_wrench.force.x = vehicle_model_snapshot_cache_.rear_right_force[0];
  tire_forces_msg.rr_wrench.force.y = vehicle_model_snapshot_cache_.rear_right_force[1];
  tire_forces_msg.rr_wrench.force.z = vehicle_model_snapshot_cache_.vertical_load.rear_right;
  tire_forces_msg.rr_wrench.torque.y = vehicle_model_snapshot_cache_.rear_right_force[2];
  tire_forces_msg.rr_wrench.torque.z = vehicle_model_snapshot_cache_.rear_right_force[3];

  tire_forces_pub_->publish(tire_forces_msg);

  tire_slip_ratio_pub_->publish(to_wheels_msg(vehicle_model_snapshot_cache_.slip_ratio, stamp));
  tire_slip_angle_pub_->publish(to_wheels_msg(vehicle_model_snapshot_cache_.slip_angle, stamp));
}

void RosOutputAdapter::publish_vm_motor(const rclcpp::Time& stamp) {
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

void RosOutputAdapter::publish_vm_battery(const rclcpp::Time& stamp) {
  custom_interfaces::msg::BatteryState battery_msg;
  battery_msg.header.stamp = stamp;
  battery_msg.header.frame_id = "base_link";
  battery_msg.voltage = vehicle_model_snapshot_cache_.battery_voltage;
  battery_msg.open_circuit_voltage = vehicle_model_snapshot_cache_.battery_open_circuit_voltage;
  battery_msg.soc = vehicle_model_snapshot_cache_.battery_soc;
  battery_msg.current = vehicle_model_snapshot_cache_.battery_current;
  battery_pub_->publish(battery_msg);
}

void RosOutputAdapter::publish_vm_transmission(const rclcpp::Time& stamp) {
  transmission_pub_->publish(
      to_wheels_msg(vehicle_model_snapshot_cache_.transmission_torque, stamp));
}

void RosOutputAdapter::publish_vm_aero(const rclcpp::Time& stamp) {
  custom_interfaces::msg::AeroForces aero_msg;
  aero_msg.header.stamp = stamp;
  aero_msg.header.frame_id = "base_link";
  aero_msg.drag = vehicle_model_snapshot_cache_.aero_drag;
  aero_msg.downforce = vehicle_model_snapshot_cache_.aero_downforce;
  aero_pub_->publish(aero_msg);
}

void RosOutputAdapter::publish_vm_status(const rclcpp::Time& stamp) {
  custom_interfaces::msg::VehicleStateVector status_msg;
  status_msg.header.stamp = stamp;
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

void RosOutputAdapter::publish_input(const rclcpp::Time& stamp) {
  const InputSnapshot input_snapshot = simulator_->get_input_snapshot();

  custom_interfaces::msg::ControlCommand input_msg;
  input_msg.header.stamp = stamp;
  input_msg.header.frame_id = "base_link";
  input_msg.throttle_fl = input_snapshot.throttle.front_left;
  input_msg.throttle_fr = input_snapshot.throttle.front_right;
  input_msg.throttle_rl = input_snapshot.throttle.rear_left;
  input_msg.throttle_rr = input_snapshot.throttle.rear_right;
  input_msg.steering = input_snapshot.steering;
  input_command_pub_->publish(input_msg);
}

void RosOutputAdapter::publish_execution_time(const rclcpp::Time& stamp) {
  custom_interfaces::msg::ExecutionTimes times_msg;
  times_msg.header.stamp = stamp;
  times_msg.header.frame_id = "base_link";
  times_msg.powertrain_ms = execution_times_snapshot_cache_.powertrain_ms;
  times_msg.transmission_ms = execution_times_snapshot_cache_.transmission_ms;
  times_msg.aero_ms = execution_times_snapshot_cache_.aero_ms;
  times_msg.steering_ms = execution_times_snapshot_cache_.steering_ms;
  times_msg.load_transfer_ms = execution_times_snapshot_cache_.load_transfer_ms;
  times_msg.tire_ms = execution_times_snapshot_cache_.tire_ms;
  times_msg.total_step_ms = execution_times_snapshot_cache_.total_step_ms;
  execution_times_pub_->publish(times_msg);
}

void RosOutputAdapter::publish_lap_summary(const rclcpp::Time& stamp) {
  if (statistics_snapshot_cache_.lap_counter <= last_published_summary_lap_) return;

  last_published_summary_lap_ = statistics_snapshot_cache_.lap_counter;

  custom_interfaces::msg::LapStatistics row_msg;
  row_msg.lap_number = statistics_snapshot_cache_.lap_counter;
  row_msg.time = statistics_snapshot_cache_.last_lap_time;
  row_msg.cones_hit = statistics_snapshot_cache_.cones_hit;
  row_msg.total_time = statistics_snapshot_cache_.total_lap_time;
  row_msg.best_time = statistics_snapshot_cache_.best_lap_time;
  row_msg.avg_velocity = statistics_snapshot_cache_.completed_lap_average_velocity * 3.6;
  row_msg.max_velocity = statistics_snapshot_cache_.completed_lap_max_velocity * 3.6;
  row_msg.avg_tracking_error_distance =
      statistics_snapshot_cache_.completed_lap_average_tracking_error;
  row_msg.max_tracking_error_distance = statistics_snapshot_cache_.completed_lap_max_tracking_error;
  row_msg.avg_velocity_error =
      statistics_snapshot_cache_.completed_lap_average_velocity_error * 3.6;
  row_msg.max_velocity_error = statistics_snapshot_cache_.completed_lap_max_velocity_error * 3.6;
  lap_summary_history_.push_back(row_msg);

  custom_interfaces::msg::LapSummary history_msg;
  history_msg.header.stamp = stamp;
  history_msg.header.frame_id = "map";
  history_msg.rows = lap_summary_history_;
  lap_summary_pub_->publish(history_msg);
}

void RosOutputAdapter::publish_lap_current(const rclcpp::Time& stamp) {
  custom_interfaces::msg::LapCurrent lap_msg;
  lap_msg.header.stamp = stamp;
  lap_msg.header.frame_id = "map";
  lap_msg.lap_number = statistics_snapshot_cache_.lap_counter + 1;
  lap_msg.current_lap_time = statistics_snapshot_cache_.current_lap_time;
  lap_msg.current_lap_cones_hit = statistics_snapshot_cache_.current_lap_cones_hit;
  lap_current_pub_->publish(lap_msg);
}

void RosOutputAdapter::publish_control_statistics(const rclcpp::Time& stamp) {
  custom_interfaces::msg::ControlStatistics statistics_msg;
  statistics_msg.header.stamp = stamp;
  statistics_msg.header.frame_id = "map";
  statistics_msg.current_velocity = statistics_snapshot_cache_.current_velocity;
  statistics_msg.current_velocity_kmh = statistics_snapshot_cache_.current_velocity * 3.6;
  statistics_msg.desired_velocity = statistics_snapshot_cache_.objective_velocity;
  statistics_msg.tracking_error = statistics_snapshot_cache_.tracking_cross_track_error;
  statistics_msg.velocity_error = statistics_snapshot_cache_.velocity_error;
  control_statistics_pub_->publish(statistics_msg);
}

void RosOutputAdapter::publish_state_estimation_velocities(const rclcpp::Time& stamp) {
  custom_interfaces::msg::Velocities vel_msg;
  vel_msg.header.stamp = stamp;
  vel_msg.header.frame_id = "base_link";
  vel_msg.velocity_x = vehicle_state_snapshot_cache_.velocity_x;
  vel_msg.velocity_y = vehicle_state_snapshot_cache_.velocity_y;
  vel_msg.angular_velocity = vehicle_state_snapshot_cache_.yaw_rate;
  std::copy(vehicle_state_snapshot_cache_.velocity_covariance.begin(),
            vehicle_state_snapshot_cache_.velocity_covariance.end(), vel_msg.covariance.begin());

  velocities_pub_->publish(vel_msg);
}

void RosOutputAdapter::publish_state_estimation_pose(const rclcpp::Time& stamp) {
  custom_interfaces::msg::Pose pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = "base_link";
  pose_msg.x = vehicle_state_snapshot_cache_.position.x;
  pose_msg.y = vehicle_state_snapshot_cache_.position.y;
  pose_msg.theta = vehicle_state_snapshot_cache_.yaw;
  std::copy(vehicle_state_snapshot_cache_.pose_covariance.begin(),
            vehicle_state_snapshot_cache_.pose_covariance.end(), pose_msg.covariance.begin());

  vehicle_pose_pub_->publish(pose_msg);
}

void RosOutputAdapter::publish_state_estimation_state_vector(const rclcpp::Time& stamp) {
  custom_interfaces::msg::VehicleStateVector state_vector_msg;
  state_vector_msg.header.stamp = stamp;
  state_vector_msg.header.frame_id = "base_link";
  state_vector_msg.velocity_x = vehicle_state_snapshot_cache_.velocity_x;
  state_vector_msg.velocity_y = vehicle_state_snapshot_cache_.velocity_y;
  state_vector_msg.yaw_rate = vehicle_state_snapshot_cache_.yaw_rate;
  state_vector_msg.acceleration_x = vehicle_state_snapshot_cache_.acceleration_x;
  state_vector_msg.acceleration_y = vehicle_state_snapshot_cache_.acceleration_y;
  state_vector_msg.steering_angle = vehicle_state_snapshot_cache_.steering_angle;

  const auto wheel_rpm = vehicle_state_snapshot_cache_.wheel_rpm;
  state_vector_msg.fl_rpm = wheel_rpm.front_left;
  state_vector_msg.fr_rpm = wheel_rpm.front_right;
  state_vector_msg.rl_rpm = wheel_rpm.rear_left;
  state_vector_msg.rr_rpm = wheel_rpm.rear_right;

  vehicle_state_vector_pub_->publish(state_vector_msg);
}

void RosOutputAdapter::publish_operational_status(const rclcpp::Time& stamp) {
  custom_interfaces::msg::OperationalStatus status_msg;
  status_msg.header.stamp = stamp;
  status_msg.header.frame_id = "base_link";
  status_msg.go_signal = vehicle_state_snapshot_cache_.go_signal;
  status_msg.as_mission = static_cast<int>(vehicle_state_snapshot_cache_.mission);
  operational_status_pub_->publish(status_msg);
}

void RosOutputAdapter::publish_visualization_car(const rclcpp::Time& stamp) {
  const double stamp_sec = stamp.seconds();
  double dt = 0.0;
  if (last_visualization_stamp_sec_ >= 0.0) {
    dt = stamp_sec - last_visualization_stamp_sec_;
  }
  last_visualization_stamp_sec_ = stamp_sec;
  if (dt < 0.0 || dt > 0.2) {
    dt = 0.0;
  }
  visualization_msgs::msg::MarkerArray vehicle_marker_array;

  add_vehicle_transform(stamp);
  add_body_marker(vehicle_marker_array, stamp);
  add_steering_marker(vehicle_marker_array, stamp);
  add_wheel_markers(vehicle_marker_array, stamp, dt);
  add_hitbox_markers(vehicle_marker_array, stamp);

  visualization_vehicle_pub_->publish(vehicle_marker_array);
}

void RosOutputAdapter::publish_visualization_ground(const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray ground_marker_array = ground_marker_template_;
  for (auto& marker : ground_marker_array.markers) {
    marker.header.stamp = stamp;
  }
  visualization_ground_pub_->publish(ground_marker_array);
}

void RosOutputAdapter::publish_visualization_gt_cones(const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray track_marker_array;
  track_marker_array = convert_cone_array_to_markers(
      mark_recently_hit_cones_red(map_snapshot_cache_.ground_truth), stamp);
  visualization_gt_cones_pub_->publish(track_marker_array);
}

void RosOutputAdapter::publish_visualization_slam_cones(const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray map_marker_array;
  map_marker_array = convert_cone_array_to_markers(
      mark_recently_hit_cones_red(map_snapshot_cache_.simulated_slam_map), stamp);
  visualization_slam_cones_pub_->publish(map_marker_array);
}

void RosOutputAdapter::publish_visualization_perception_cones(const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray perception_marker_array;
  perception_marker_array =
      convert_cone_array_to_markers(map_snapshot_cache_.perception_cones, stamp, "car");
  visualization_perception_cones_pub_->publish(perception_marker_array);
}

void RosOutputAdapter::load_cone_visualization_resources() {
  const std::string path =
      std::string(INVICTASIM_SOURCE_DIR) + "/resources/meshes/cones/config.yaml";
  const YAML::Node config = YAML::LoadFile(path);

  double hitbox_z = 0.02;
  double hitbox_height = 0.04;
  double hitbox_alpha = 0.35;
  const YAML::Node hitboxes = config["visualization"]["hitboxes"];
  if (hitboxes) {
    visualize_cone_hitboxes_ = hitboxes["visualize"].as<bool>(visualize_cone_hitboxes_);
    hitbox_z = hitboxes["z"].as<double>(hitbox_z);
    hitbox_height = hitboxes["height"].as<double>(hitbox_height);
    hitbox_alpha = hitboxes["alpha"].as<double>(hitbox_alpha);
  }

  const YAML::Node collision = config["collision"];
  if (collision) {
    cone_standard_radius_ = collision["standard_radius"].as<double>(cone_standard_radius_);
    cone_large_radius_ = collision["large_radius"].as<double>(cone_large_radius_);
    cone_hit_match_distance_ =
        collision["hit_match_distance"].as<double>(cone_hit_match_distance_);
  }

  cone_hitbox_marker_template_.ns = "cone_hitboxes";
  cone_hitbox_marker_template_.type = visualization_msgs::msg::Marker::CYLINDER;
  cone_hitbox_marker_template_.action = visualization_msgs::msg::Marker::ADD;
  cone_hitbox_marker_template_.pose.position.z = hitbox_z;
  cone_hitbox_marker_template_.pose.orientation.w = 1.0;
  cone_hitbox_marker_template_.scale.z = hitbox_height;
  cone_hitbox_marker_template_.color.r = 0.0f;
  cone_hitbox_marker_template_.color.g = 0.8f;
  cone_hitbox_marker_template_.color.b = 1.0f;
  cone_hitbox_marker_template_.color.a = static_cast<float>(hitbox_alpha);
}

void RosOutputAdapter::load_ground_visualization_resources() {
  const std::string path =
      std::string(INVICTASIM_SOURCE_DIR) + "/resources/meshes/ground/config.yaml";
  const YAML::Node config = YAML::LoadFile(path);
  double position_x = 0.0;
  double position_y = 0.0;
  double position_z = -0.02;
  double scale_x = 1000.0;
  double scale_y = 1000.0;
  double scale_z = 1.0;
  double color_r = 0.78;
  double color_g = 0.78;
  double color_b = 0.78;
  double color_a = 1.0;
  double timing_line_target_cell_length = 0.5;
  int timing_line_row_count = 2;
  double timing_line_total_width = 0.45;
  double timing_line_z = 0.01;
  double timing_line_height = 0.02;

  const YAML::Node visualization = config["visualization"];
  if (visualization) {
    const YAML::Node position = visualization["position"];
    if (position) {
      position_x = position["x"].as<double>(position_x);
      position_y = position["y"].as<double>(position_y);
      position_z = position["z"].as<double>(position_z);
    }

    const YAML::Node scale = visualization["scale"];
    if (scale) {
      scale_x = scale["x"].as<double>(scale_x);
      scale_y = scale["y"].as<double>(scale_y);
      scale_z = scale["z"].as<double>(scale_z);
    }

    const YAML::Node color = visualization["color"];
    if (color) {
      color_r = color["r"].as<double>(color_r);
      color_g = color["g"].as<double>(color_g);
      color_b = color["b"].as<double>(color_b);
      color_a = color["a"].as<double>(color_a);
    }

    const YAML::Node timing_line = visualization["timing_line"];
    if (timing_line) {
      timing_line_target_cell_length =
          timing_line["target_cell_length"].as<double>(timing_line_target_cell_length);
      timing_line_row_count = timing_line["row_count"].as<int>(timing_line_row_count);
      timing_line_total_width = timing_line["total_width"].as<double>(timing_line_total_width);
      timing_line_z = timing_line["z"].as<double>(timing_line_z);
      timing_line_height = timing_line["height"].as<double>(timing_line_height);
    }
  }

  visualization_msgs::msg::Marker ground;
  ground.header.frame_id = "map";
  ground.ns = "invictasim_ground";
  ground.id = 100;
  ground.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  ground.action = visualization_msgs::msg::Marker::ADD;
  ground.pose.position.x = position_x;
  ground.pose.position.y = position_y;
  ground.pose.position.z = position_z;
  ground.pose.orientation.w = 1.0;
  ground.scale.x = scale_x;
  ground.scale.y = scale_y;
  ground.scale.z = scale_z;
  ground.color.r = static_cast<float>(color_r);
  ground.color.g = static_cast<float>(color_g);
  ground.color.b = static_cast<float>(color_b);
  ground.color.a = static_cast<float>(color_a);
  ground.mesh_resource = "package://invictasim/resources/meshes/ground/ground_plane.dae";
  ground.mesh_use_embedded_materials = true;

  ground_marker_template_.markers.clear();
  ground_marker_template_.markers.push_back(ground);
  add_timing_line_markers(ground_marker_template_, timing_line_target_cell_length,
                          timing_line_row_count, timing_line_total_width, timing_line_z,
                          timing_line_height);
}

visualization_msgs::msg::MarkerArray RosOutputAdapter::convert_cone_array_to_markers(
    const std::vector<common_lib::structures::Cone>& cone_array, const rclcpp::Time& stamp,
    const std::string& frame_id) const {
  visualization_msgs::msg::MarkerArray marker_array;
  int cone_id = 0;
  static const std::string cone_mesh_path = "package://invictasim/resources/meshes/cones/";
  for (const auto& cone : cone_array) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    if (frame_id == "car") {
      m.frame_locked = true;
      m.lifetime = rclcpp::Duration::from_seconds(0.1);
    }
    m.ns = "cones";
    const int current_cone_id = cone_id++;
    m.id = current_cone_id;
    m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = cone.position.x;
    m.pose.position.y = cone.position.y;
    m.pose.position.z = 0.1425;

    m.pose.orientation.w = 1.0;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.mesh_use_embedded_materials = true;
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
    m.color.a = 1.0f;

    switch (cone.color) {
      case common_lib::competition_logic::Color::BLUE:
        m.mesh_resource = cone_mesh_path + "cone_blue.dae";
        break;
      case common_lib::competition_logic::Color::YELLOW:
        m.mesh_resource = cone_mesh_path + "cone_yellow.dae";
        break;
      case common_lib::competition_logic::Color::LARGE_ORANGE:
        m.mesh_resource = cone_mesh_path + "cone_orange_big.dae";
        m.pose.position.z = 0.03;  // Adjusted height for large cone
        break;
      case common_lib::competition_logic::Color::RED:
        m.mesh_resource = cone_mesh_path + "cone_red.dae";
        break;
      case common_lib::competition_logic::Color::GREEN:
        m.mesh_resource = cone_mesh_path + "cone_green.dae";
        break;
      case common_lib::competition_logic::Color::ORANGE:
      default:
        m.mesh_resource = cone_mesh_path + "cone_orange.dae";
        break;
    }
    marker_array.markers.push_back(m);
    if (visualize_cone_hitboxes_) {
      const bool is_large =
          cone.is_large || cone.color == common_lib::competition_logic::Color::LARGE_ORANGE;
      const double radius = is_large ? cone_large_radius_ : cone_standard_radius_;
      visualization_msgs::msg::Marker hitbox = cone_hitbox_marker_template_;
      hitbox.header = m.header;
      hitbox.frame_locked = m.frame_locked;
      hitbox.lifetime = m.lifetime;
      hitbox.id = 10000 + current_cone_id;
      hitbox.pose.position.x = cone.position.x;
      hitbox.pose.position.y = cone.position.y;
      hitbox.scale.x = radius * 2.0;
      hitbox.scale.y = radius * 2.0;
      marker_array.markers.push_back(hitbox);
    }
  }
  return marker_array;
}

std::vector<common_lib::structures::Cone> RosOutputAdapter::mark_recently_hit_cones_red(
    std::vector<common_lib::structures::Cone> cones) const {
  for (auto& cone : cones) {
    for (const auto& hit_cone : map_snapshot_cache_.recently_hit_cones) {
      if (cone.position.euclidean_distance(hit_cone.position) <= cone_hit_match_distance_) {
        cone.color = common_lib::competition_logic::Color::RED;
        break;
      }
    }
  }
  return cones;
}

std::string RosOutputAdapter::get_car_mesh_resource(const std::string& mesh_name) const {
  return "package://invictasim/resources/meshes/car/" +
         simulator_->get_params().car_parameters_config + "/" + mesh_name;
}

void RosOutputAdapter::load_car_visualization_resources() {
  const std::string car_folder = simulator_->get_params().car_parameters_config;
  const std::string pos_file =
      std::string(INVICTASIM_SOURCE_DIR) + "/resources/meshes/car/" + car_folder + "/config.yaml";
  const YAML::Node config = YAML::LoadFile(pos_file);
  const auto positions = load_car_mesh_positions(config);
  const auto car_params = simulator_->get_params().car_parameters;
  const double wheel_center_z = car_params->wheel_diameter * 0.5;
  const double front_axle_x = car_params->wheelbase - car_params->cg_2_rear_axis;
  const double rear_axle_x = -car_params->cg_2_rear_axis;
  const double half_track = car_params->track_width * 0.5;

  body_marker_template_.header.frame_id = "car";
  body_marker_template_.frame_locked = true;
  body_marker_template_.ns = "invictasim_vehicle";
  body_marker_template_.id = 0;
  body_marker_template_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  body_marker_template_.action = visualization_msgs::msg::Marker::ADD;
  body_marker_template_.pose.orientation.w = 1.0;
  body_marker_template_.scale.x = 1.0;
  body_marker_template_.scale.y = 1.0;
  body_marker_template_.scale.z = 1.0;
  body_marker_template_.color.a = 1.0f;
  body_marker_template_.color.r = 0.85f;
  body_marker_template_.mesh_resource = get_car_mesh_resource("car_body.glb");
  body_marker_template_.mesh_use_embedded_materials = true;

  steering_marker_template_.header.frame_id = "car";
  steering_marker_template_.frame_locked = true;
  steering_marker_template_.ns = "invictasim_vehicle";
  steering_marker_template_.id = 5;
  steering_marker_template_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  steering_marker_template_.action = visualization_msgs::msg::Marker::ADD;
  steering_marker_template_.pose.position.x = positions[0];
  steering_marker_template_.pose.position.y = positions[1];
  steering_marker_template_.pose.position.z = positions[2];
  tf2::Quaternion steering_mount;
  steering_mount.setRPY(positions[3], positions[4], positions[5]);
  steering_marker_template_.pose.orientation.x = steering_mount.x();
  steering_marker_template_.pose.orientation.y = steering_mount.y();
  steering_marker_template_.pose.orientation.z = steering_mount.z();
  steering_marker_template_.pose.orientation.w = steering_mount.w();
  steering_rotation_multiplier_ = positions[9];
  steering_marker_template_.scale.x = 1.0;
  steering_marker_template_.scale.y = 1.0;
  steering_marker_template_.scale.z = 1.0;
  steering_marker_template_.color.a = 1.0f;
  steering_marker_template_.mesh_resource = get_car_mesh_resource("steering.glb");
  steering_marker_template_.mesh_use_embedded_materials = false;

  const double local_x[4] = {front_axle_x, front_axle_x, rear_axle_x, rear_axle_x};
  const double local_y[4] = {half_track, -half_track, half_track, -half_track};
  for (int i = 0; i < 4; ++i) {
    auto& wheel = wheel_marker_templates_[i];
    wheel.header.frame_id = "car";
    wheel.frame_locked = true;
    wheel.ns = "invictasim_vehicle";
    wheel.id = i + 1;
    wheel.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    wheel.action = visualization_msgs::msg::Marker::ADD;
    wheel.pose.position.x = local_x[i] + positions[6];
    wheel.pose.position.y = local_y[i] + positions[7];
    wheel.pose.position.z = wheel_center_z + positions[8];
    wheel.scale.x = 1.0;
    wheel.scale.y = 1.0;
    wheel.scale.z = 1.0;
    wheel.color.a = 1.0f;
    wheel.mesh_resource = get_car_mesh_resource(i < 2 ? "wheel_front.glb" : "wheel_back.glb");
    wheel.mesh_use_embedded_materials = false;
  }

  const YAML::Node yaml_hitboxes = config["hitboxes"];
  if (!yaml_hitboxes || !yaml_hitboxes["visualize"].as<bool>(false)) {
    return;
  }

  const YAML::Node yaml_boxes = yaml_hitboxes["boxes"];
  if (!yaml_boxes || !yaml_boxes.IsSequence()) {
    return;
  }

  car_hitbox_marker_templates_.clear();
  for (const auto& node : yaml_boxes) {
    const double length = node["length"].as<double>(0.0);
    const double width = node["width"].as<double>(0.0);
    if (length <= 0.0 || width <= 0.0) {
      continue;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "car";
    marker.frame_locked = true;
    marker.ns = "invictasim_vehicle_hitboxes";
    marker.id = static_cast<int>(car_hitbox_marker_templates_.size());
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = node["center_x"].as<double>(0.0);
    marker.pose.position.y = node["center_y"].as<double>(0.0);
    marker.pose.position.z = 0.12;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = length;
    marker.scale.y = width;
    marker.scale.z = 0.08;
    marker.color.a = 0.28f;
    marker.color.g = 0.8f;
    marker.color.b = 1.0f;
    car_hitbox_marker_templates_.push_back(marker);
  }
}

std::array<double, 10> RosOutputAdapter::load_car_mesh_positions(const YAML::Node& config) {
  std::array<double, 10> positions{};
  const YAML::Node yaml_positions = config["positions"];
  if (!yaml_positions) {
    return positions;
  }

  positions[0] = yaml_positions["steering_offset_x"].as<double>(positions[0]);
  positions[1] = yaml_positions["steering_offset_y"].as<double>(positions[1]);
  positions[2] = yaml_positions["steering_offset_z"].as<double>(positions[2]);
  positions[3] = yaml_positions["steering_rotation_x"].as<double>(positions[3]);
  positions[4] = yaml_positions["steering_rotation_y"].as<double>(positions[4]);
  positions[5] = yaml_positions["steering_rotation_z"].as<double>(positions[5]);
  positions[6] = yaml_positions["wheels_offset_x"].as<double>(positions[6]);
  positions[7] = yaml_positions["wheels_offset_y"].as<double>(positions[7]);
  positions[8] = yaml_positions["wheels_offset_z"].as<double>(positions[8]);
  positions[9] = yaml_positions["steering_rotation_multiplier"].as<double>(positions[9]);
  positions[3] *= M_PI / 180.0;  // Convert from degrees to radians
  positions[4] *= M_PI / 180.0;
  positions[5] *= M_PI / 180.0;

  return positions;
}

std::vector<RosOutputAdapter::TimingLine> RosOutputAdapter::make_timing_lines() const {
  std::string discipline = simulator_->get_discipline();
  std::transform(discipline.begin(), discipline.end(), discipline.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (discipline == "acceleration") {
    return make_acceleration_timing_lines();
  }
  return make_default_timing_lines();
}

std::vector<RosOutputAdapter::TimingLine> RosOutputAdapter::make_default_timing_lines() const {
  const auto timing_line = simulator_->get_timing_line();
  return {{timing_line.first, timing_line.second}};
}

std::vector<RosOutputAdapter::TimingLine> RosOutputAdapter::make_acceleration_timing_lines() const {
  std::vector<TimingLine> timing_lines;
  const auto& configured_timing_lines = simulator_->get_timing_lines();
  for (const auto& timing_line : configured_timing_lines) {
    timing_lines.emplace_back(timing_line.first, timing_line.second);
  }
  return timing_lines;
}

void RosOutputAdapter::add_timing_line_markers(visualization_msgs::msg::MarkerArray& marker_array,
                                               double target_cell_length, int row_count,
                                               double total_width, double z, double height) const {
  const auto timing_lines = make_timing_lines();
  const double cell_length_target = std::max(0.01, target_cell_length);
  const int rows = std::max(1, row_count);

  for (std::size_t i = 0; i < timing_lines.size(); ++i) {
    const auto& start = std::get<0>(timing_lines[i]);
    const auto& end = std::get<1>(timing_lines[i]);
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length = std::hypot(dx, dy);
    if (length <= std::numeric_limits<double>::epsilon()) {
      continue;
    }

    const double yaw = std::atan2(dy, dx);
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);

    const int id_offset = static_cast<int>(i) * 1000;
    const int column_count = std::max(2, static_cast<int>(std::ceil(length / cell_length_target)));
    const double cell_length = length / static_cast<double>(column_count);
    const double cell_width = total_width / static_cast<double>(rows);

    for (int column = 0; column < column_count; ++column) {
      for (int row = 0; row < rows; ++row) {
        visualization_msgs::msg::Marker cell;
        cell.header.frame_id = "map";
        cell.ns = "invictasim_timing_line";
        cell.id = 200 + id_offset + column * rows + row;
        cell.type = visualization_msgs::msg::Marker::CUBE;
        cell.action = visualization_msgs::msg::Marker::ADD;

        const double t = (static_cast<double>(column) + 0.5) / static_cast<double>(column_count);
        const double lateral_offset =
            (static_cast<double>(row) + 0.5 - static_cast<double>(rows) * 0.5) * cell_width;
        cell.pose.position.x = start.x + t * dx - std::sin(yaw) * lateral_offset;
        cell.pose.position.y = start.y + t * dy + std::cos(yaw) * lateral_offset;
        cell.pose.position.z = z;
        cell.pose.orientation.x = orientation.x();
        cell.pose.orientation.y = orientation.y();
        cell.pose.orientation.z = orientation.z();
        cell.pose.orientation.w = orientation.w();

        cell.scale.x = cell_length;
        cell.scale.y = cell_width;
        cell.scale.z = height;
        cell.color.a = 1.0f;
        const bool is_white = ((column + row) % 2) == 0;
        cell.color.r = is_white ? 1.0f : 0.02f;
        cell.color.g = is_white ? 1.0f : 0.02f;
        cell.color.b = is_white ? 1.0f : 0.02f;
        marker_array.markers.push_back(cell);
      }
    }
  }
}

void RosOutputAdapter::add_body_marker(visualization_msgs::msg::MarkerArray& marker_array,
                                       const rclcpp::Time& stamp) const {
  visualization_msgs::msg::Marker body = body_marker_template_;
  body.header.stamp = stamp;
  marker_array.markers.push_back(body);
}

void RosOutputAdapter::add_hitbox_markers(visualization_msgs::msg::MarkerArray& marker_array,
                                          const rclcpp::Time& stamp) const {
  for (auto marker : car_hitbox_marker_templates_) {
    marker.header.stamp = stamp;
    marker_array.markers.push_back(marker);
  }
}

void RosOutputAdapter::add_steering_marker(visualization_msgs::msg::MarkerArray& marker_array,
                                           const rclcpp::Time& stamp) const {
  const auto& mount = steering_marker_template_.pose.orientation;
  tf2::Quaternion q_mount(mount.x, mount.y, mount.z, mount.w);

  tf2::Quaternion q_steering;
  q_steering.setRPY(-vehicle_model_snapshot_cache_.steering_angle * steering_rotation_multiplier_,
                    0.0, 0.0);

  tf2::Quaternion q_total = q_mount * q_steering;
  q_total.normalize();

  visualization_msgs::msg::Marker steering = steering_marker_template_;
  steering.header.stamp = stamp;
  steering.pose.orientation.x = q_total.x();
  steering.pose.orientation.y = q_total.y();
  steering.pose.orientation.z = q_total.z();
  steering.pose.orientation.w = q_total.w();

  marker_array.markers.push_back(steering);
}

void RosOutputAdapter::add_wheel_markers(visualization_msgs::msg::MarkerArray& marker_array,
                                         const rclcpp::Time& stamp, double dt) {
  if (dt > 0.0) {
    const auto wheel_speed = vehicle_model_snapshot_cache_.wheel_speed;
    wheel_spin_fl_ += (wheel_speed.front_left) * dt;
    wheel_spin_fr_ += (wheel_speed.front_right) * dt;
    wheel_spin_rl_ += (wheel_speed.rear_left) * dt;
    wheel_spin_rr_ += (wheel_speed.rear_right) * dt;
  }

  const double steer = vehicle_model_snapshot_cache_.steering_angle;
  const double steer_angles[4] = {steer, steer, 0.0, 0.0};
  const double spins[4] = {wheel_spin_fl_, wheel_spin_fr_, wheel_spin_rl_, wheel_spin_rr_};

  for (int i = 0; i < 4; ++i) {
    // Rotation logic in the local frame:
    tf2::Quaternion q_steer;
    q_steer.setRPY(0.0, 0.0, steer_angles[i]);

    tf2::Quaternion q_spin;
    q_spin.setRPY(0.0, spins[i], 0.0);

    tf2::Quaternion q_side_offset;
    q_side_offset.setRPY((i == 0 || i == 2) ? M_PI : 0.0, 0.0, 0.0);

    // Mirror the left-side mesh first, then apply visual roll and steering.
    tf2::Quaternion q_wheel = q_steer * q_spin * q_side_offset;
    q_wheel.normalize();

    visualization_msgs::msg::Marker wheel = wheel_marker_templates_[i];
    wheel.header.stamp = stamp;
    wheel.pose.orientation.x = q_wheel.x();
    wheel.pose.orientation.y = q_wheel.y();
    wheel.pose.orientation.z = q_wheel.z();
    wheel.pose.orientation.w = q_wheel.w();

    marker_array.markers.push_back(wheel);
  }
}

void RosOutputAdapter::add_vehicle_transform(const rclcpp::Time& stamp) {
  geometry_msgs::msg::TransformStamped car_transform;
  car_transform.header.stamp = stamp;
  car_transform.header.frame_id = "map";
  car_transform.child_frame_id = "car";
  car_transform.transform.translation.x = vehicle_model_snapshot_cache_.x;
  car_transform.transform.translation.y = vehicle_model_snapshot_cache_.y;
  car_transform.transform.translation.z = 0.0;

  tf2::Quaternion car_rotation;
  car_rotation.setRPY(0.0, 0.0, vehicle_model_snapshot_cache_.yaw);
  car_transform.transform.rotation.x = car_rotation.x();
  car_transform.transform.rotation.y = car_rotation.y();
  car_transform.transform.rotation.z = car_rotation.z();
  car_transform.transform.rotation.w = car_rotation.w();

  tf_broadcaster_->sendTransform(car_transform);
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
