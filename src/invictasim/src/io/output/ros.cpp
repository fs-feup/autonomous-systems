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

  // Visualization Publishers
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  visualization_ground_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "invictasim/visualization/ground", 10);
  visualization_vehicle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "invictasim/visualization/vehicle", 10);
  visualization_gt_cones_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "invictasim/visualization/ground_truth_cones", 10);

  // Simulated perception publishers
  if (simulator_->get_params().use_simulated_perception) {
    visualization_perception_cones_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "invictasim/visualization/perception_cones", 10);
    perception_pub_ = this->create_publisher<custom_interfaces::msg::PerceptionOutput>(
        "invictasim/perception/cones", 10);
  }

  // Simulated state estimation publishers
  if (simulator_->get_params().use_simulated_se) {
    state_map_pub_ = this->create_publisher<custom_interfaces::msg::ConeArray>(
        "invictasim/state_estimation/map", 10);
    visualization_slam_cones_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "invictasim/visualization/slam_cones", 10);
    vehicle_pose_pub_ = this->create_publisher<custom_interfaces::msg::Pose>(
        "invictasim/state_estimation/vehicle_pose", 10);
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

  if (config["publish_frequencies"]["execution_time"]) {
    topic_frequencies_["execution_time"] =
        config["publish_frequencies"]["execution_time"].as<int>();
  }

  // After loading the config, build the dispatch table
  map_callbacks();
}

void RosOutputAdapter::map_callbacks() {
  // Vehicle model
  register_pub_helper("tire", [this](const rclcpp::Time& stamp) { publish_vm_tire(stamp); });
  register_pub_helper("motor", [this](const rclcpp::Time& stamp) { publish_vm_motor(stamp); });
  register_pub_helper("battery", [this](const rclcpp::Time& stamp) { publish_vm_battery(stamp); });
  register_pub_helper("transmission",
                      [this](const rclcpp::Time& stamp) { publish_vm_transmission(stamp); });
  register_pub_helper("aero", [this](const rclcpp::Time& stamp) { publish_vm_aero(stamp); });
  register_pub_helper("status", [this](const rclcpp::Time& stamp) {
    publish_vm_status(stamp);
    publish_input(stamp);
  });

  // Visualization
  register_pub_helper("car",
                      [this](const rclcpp::Time& stamp) { publish_visualization_car(stamp); });
  register_pub_helper("ground",
                      [this](const rclcpp::Time& stamp) { publish_visualization_ground(stamp); });
  register_pub_helper("ground_truth_cones",
                      [this](const rclcpp::Time& stamp) { publish_visualization_gt_cones(stamp); });

  // Sensors
  register_pub_helper("imu", [this](const rclcpp::Time& stamp) { publish_sensors_imu(stamp); });
  register_pub_helper("wheel_speed",
                      [this](const rclcpp::Time& stamp) { publish_sensors_wheel_speed(stamp); });
  register_pub_helper("resolver",
                      [this](const rclcpp::Time& stamp) { publish_sensors_resolver(stamp); });
  register_pub_helper("steering",
                      [this](const rclcpp::Time& stamp) { publish_sensors_steering(stamp); });

  // Map
  register_pub_helper("ground_truth",
                      [this](const rclcpp::Time& stamp) { publish_map_ground_truth(stamp); });

  // Operational status
  register_pub_helper("operational_status",
                      [this](const rclcpp::Time& stamp) { publish_operational_status(stamp); });

  // Exec times
  register_pub_helper("execution_time",
                      [this](const rclcpp::Time& stamp) { publish_execution_time(stamp); });

  // Simulated state estimation
  if (simulator_->get_params().use_simulated_se) {
    register_pub_helper("slam_cones", [this](const rclcpp::Time& stamp) {
      publish_visualization_slam_cones(stamp);
    });
    register_pub_helper("simulated_slam", [this](const rclcpp::Time& stamp) {
      publish_state_estimation_map(stamp);
      publish_state_estimation_lap_counter();
    });
    register_pub_helper(
        "pose", [this](const rclcpp::Time& stamp) { publish_state_estimation_pose(stamp); });
  }

  // Simulated perception
  if (simulator_->get_params().use_simulated_perception) {
    register_pub_helper("perception_cones",
                        [this](const rclcpp::Time& stamp) { publish_perception_cones(stamp); });
    register_pub_helper("perception_cones", [this](const rclcpp::Time& stamp) {
      publish_visualization_perception_cones(stamp);
    });
  }

  // Simulated velocities
  if (simulator_->get_params().use_simulated_velocities) {
    register_pub_helper("velocities", [this](const rclcpp::Time& stamp) {
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
                                           std::function<void(const rclcpp::Time&)> func) {
  if (topic_frequencies_.count(topic) && topic_frequencies_[topic] > 0) {
    frequency_callbacks_[topic_frequencies_[topic]].push_back(func);
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

void RosOutputAdapter::on_frequency_tick(int frequency_hz) {
  const rclcpp::Time stamp = this->now();

  // Refresh snapshots
  refresh_vehicle_model_snapshot();
  refresh_execution_times_snapshot();
  refresh_map_snapshot();
  refresh_sensors_snapshot();
  refresh_vehicle_state_snapshot();

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
  const auto& cones = map_snapshot_cache_.ground_truth;
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
  const auto& cones = map_snapshot_cache_.simulated_slam_map;
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
  lap_msg.data = static_cast<double>(map_snapshot_cache_.lap_counter);
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
  add_wheel_markers(vehicle_marker_array, stamp, dt);

  visualization_vehicle_pub_->publish(vehicle_marker_array);
}

void RosOutputAdapter::publish_visualization_ground(const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray ground_marker_array;
  visualization_msgs::msg::Marker ground;
  ground.header.stamp = stamp;
  ground.header.frame_id = "map";
  ground.ns = "invictasim_ground";
  ground.id = 100;
  ground.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  ground.action = visualization_msgs::msg::Marker::ADD;
  ground.pose.position.x = 0.0;
  ground.pose.position.y = 0.0;
  ground.pose.position.z = -0.02;
  ground.pose.orientation.x = 0.0;
  ground.pose.orientation.y = 0.0;
  ground.pose.orientation.z = 0.0;
  ground.pose.orientation.w = 1.0;
  ground.scale.x = 1000.0;
  ground.scale.y = 1000.0;
  ground.scale.z = 1.0;
  ground.color.a = 1.0f;
  ground.color.r = 0.78f;
  ground.color.g = 0.78f;
  ground.color.b = 0.78f;
  ground.mesh_resource = "package://invictasim/resources/meshes/ground_plane.dae";
  ground.mesh_use_embedded_materials = true;
  ground_marker_array.markers.push_back(ground);
  visualization_ground_pub_->publish(ground_marker_array);
}

void RosOutputAdapter::publish_visualization_gt_cones(const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray track_marker_array;
  track_marker_array = convert_cone_array_to_markers(map_snapshot_cache_.ground_truth, stamp);
  visualization_gt_cones_pub_->publish(track_marker_array);
}

void RosOutputAdapter::publish_visualization_slam_cones(const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray map_marker_array;
  map_marker_array = convert_cone_array_to_markers(map_snapshot_cache_.simulated_slam_map, stamp);
  visualization_slam_cones_pub_->publish(map_marker_array);
}

void RosOutputAdapter::publish_visualization_perception_cones(const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray perception_marker_array;
  perception_marker_array =
      convert_cone_array_to_markers(map_snapshot_cache_.perception_cones, stamp);
  visualization_perception_cones_pub_->publish(perception_marker_array);
}

visualization_msgs::msg::MarkerArray RosOutputAdapter::convert_cone_array_to_markers(
    std::vector<common_lib::structures::Cone>& cone_array, const rclcpp::Time& stamp) const {
  visualization_msgs::msg::MarkerArray marker_array;
  int cone_id = 0;
  for (const auto& cone : cone_array) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = "map";
    m.ns = "cones";
    m.id = cone_id++;
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

    std::string path = "package://invictasim/resources/meshes/cones/";

    switch (cone.color) {
      case common_lib::competition_logic::Color::BLUE:
        m.mesh_resource = path + "cone_blue.dae";
        break;
      case common_lib::competition_logic::Color::YELLOW:
        m.mesh_resource = path + "cone_yellow.dae";
        break;
      case common_lib::competition_logic::Color::LARGE_ORANGE:
        m.mesh_resource = path + "cone_orange_big.dae";
        m.pose.position.z = 0.03;  // Adjusted height for large cone
        break;
      case common_lib::competition_logic::Color::ORANGE:
      default:
        m.mesh_resource = path + "cone_orange.dae";
        break;
    }
    marker_array.markers.push_back(m);
  }
  return marker_array;
}

void RosOutputAdapter::add_body_marker(visualization_msgs::msg::MarkerArray& marker_array,
                                       const rclcpp::Time& stamp) const {
  tf2::Quaternion q_mesh_offset;
  q_mesh_offset.setRPY(-M_PI_2, 0.0, M_PI_2);

  visualization_msgs::msg::Marker body;
  body.header.stamp = stamp;
  body.header.frame_id = "car";
  body.frame_locked = true;
  body.ns = "invictasim_vehicle";
  body.id = 0;
  body.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  body.action = visualization_msgs::msg::Marker::ADD;

  body.pose.position.x = 1.0;
  body.pose.position.y = 0.0;
  body.pose.position.z = 0.0;

  body.pose.orientation.x = q_mesh_offset.x();
  body.pose.orientation.y = q_mesh_offset.y();
  body.pose.orientation.z = q_mesh_offset.z();
  body.pose.orientation.w = q_mesh_offset.w();

  body.scale.x = 1.0;
  body.scale.y = 1.0;
  body.scale.z = 1.0;
  body.color.a = 1.0f;
  body.color.r = 0.85f;
  body.color.g = 0.1f;
  body.color.b = 0.1f;
  body.mesh_resource = "package://invictasim/resources/meshes/car_body.stl";
  body.mesh_use_embedded_materials = false;

  marker_array.markers.push_back(body);
}

void RosOutputAdapter::add_wheel_markers(visualization_msgs::msg::MarkerArray& marker_array,
                                         const rclcpp::Time& stamp, double dt) {
  const auto car_params = simulator_->get_params().car_parameters;
  const double wheel_center_z = car_params->wheel_diameter * 0.5;
  const double long_offset = 0.15;

  if (dt > 0.0) {
    const auto wheel_speed = vehicle_model_snapshot_cache_.wheel_speed;
    wheel_spin_fl_ += (wheel_speed.front_left) * dt;
    wheel_spin_fr_ += (wheel_speed.front_right) * dt;
    wheel_spin_rl_ += (wheel_speed.rear_left) * dt;
    wheel_spin_rr_ += (wheel_speed.rear_right) * dt;
  }

  const double steer = vehicle_model_snapshot_cache_.steering_angle;

  const double front_axle_x = car_params->wheelbase - car_params->cg_2_rear_axis + long_offset;
  const double rear_axle_x = -car_params->cg_2_rear_axis + long_offset;
  const double half_track = car_params->track_width * 0.5;

  const double local_x[4] = {front_axle_x, front_axle_x, rear_axle_x, rear_axle_x};
  const double local_y[4] = {half_track, -half_track, half_track, -half_track};
  const double steer_angles[4] = {steer, steer, 0.0, 0.0};
  const double spins[4] = {wheel_spin_fl_, wheel_spin_fr_, wheel_spin_rl_, wheel_spin_rr_};

  for (int i = 0; i < 4; ++i) {
    // Rotation logic in the local frame:
    tf2::Quaternion q_steer;
    q_steer.setRPY(0.0, 0.0, steer_angles[i]);

    tf2::Quaternion q_spin;
    q_spin.setRPY(0.0, spins[i], 0.0);

    tf2::Quaternion q_mesh_offset;
    q_mesh_offset.setRPY(-M_PI_2, 0.0, 0.0);

    tf2::Quaternion q_side_offset;
    q_side_offset.setRPY(0.0, 0.0, (i == 1 || i == 3) ? M_PI : 0.0);

    // Order: apply steering, then wheel spin, then mesh corrections
    tf2::Quaternion q_wheel = q_steer * q_spin * q_side_offset * q_mesh_offset;
    q_wheel.normalize();

    visualization_msgs::msg::Marker wheel;
    wheel.header.stamp = stamp;
    wheel.frame_locked = true;
    wheel.header.frame_id = "car";  // Locked to the moving car frame
    wheel.ns = "invictasim_vehicle";
    wheel.id = i + 1;
    wheel.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    wheel.action = visualization_msgs::msg::Marker::ADD;

    // Use pure local offsets
    wheel.pose.position.x = local_x[i];
    wheel.pose.position.y = local_y[i];
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