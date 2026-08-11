#include "io/output/ros.hpp"
#include <iomanip>
#include <sstream>
#include <fstream>
#include <filesystem>

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
  steering_pub_ = 
      this->create_publisher<custom_interfaces::msg::SteeringAngle>("invictasim/steering_sensor", 10);

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
  register_pub_helper("ground", refresh_map,
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
    register_pub_helper("pose", refresh_vehicle_state, [this](const rclcpp::Time& stamp) {
      publish_state_estimation_pose(stamp);
    });
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
    frequency_callbacks_[frequency].push_back([refresh_snapshot, func](const rclcpp::Time& stamp) {
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
  times_msg.integration_ms = execution_times_snapshot_cache_.integration_ms;
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

std::string RosOutputAdapter::ensure_grass_plane_mesh(double size_x, double size_y) const {
  // One tile per grass_tile_size_m_ metres in each direction.
  const double tile = std::max(0.05, grass_tile_size_m_);
  const double repeat_u = std::max(1.0, size_x / tile);
  const double repeat_v = std::max(1.0, size_y / tile);

  std::ostringstream name;
  name << "grass_plane_" << std::fixed << std::setprecision(0) << size_x << "x" << size_y;
  const std::string stem = name.str();
  // Written into the package share directory, because that is what package://
  // resolves to. Writing into the source tree does not work: --symlink-install
  // links each file individually at build time, so a file created afterwards has
  // no link in the share directory and the viewer cannot fetch it.
  const std::string fallback = "package://invictasim/resources/meshes/ground/grass_plane.dae";
  std::string share_directory;
  try {
    share_directory = ament_index_cpp::get_package_share_directory("invictasim");
  } catch (const std::exception&) {
    return fallback;
  }
  const std::string directory = share_directory + "/resources/meshes/ground/";
  const std::string file_path = directory + stem + ".dae";
  const std::string resource = "package://invictasim/resources/meshes/ground/" + stem + ".dae";

  std::error_code ec;
  if (std::filesystem::exists(file_path, ec)) {
    return resource;
  }

  std::ostringstream uv;
  uv << std::fixed << std::setprecision(4) << "0 0 " << repeat_u << " 0 " << repeat_u << " "
     << repeat_v << " 0 " << repeat_v;

  std::ostringstream dae;
  dae << R"(<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>InvictaSim generated grass plane</authoring_tool></contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_images>
    <image id="grass_texture-image" name="grass_texture">
      <init_from>grass_texture.png</init_from>
    </image>
  </library_images>
  <library_effects>
    <effect id="grass_effect">
      <profile_COMMON>
        <newparam sid="grass_surface">
          <surface type="2D"><init_from>grass_texture-image</init_from></surface>
        </newparam>
        <newparam sid="grass_sampler">
          <sampler2D>
            <source>grass_surface</source>
            <wrap_s>WRAP</wrap_s>
            <wrap_t>WRAP</wrap_t>
            <minfilter>LINEAR_MIPMAP_LINEAR</minfilter>
            <magfilter>LINEAR</magfilter>
          </sampler2D>
        </newparam>
        <technique sid="common">
          <phong>
            <emission><color>0 0 0 1</color></emission>
            <ambient><color>0.02 0.02 0.02 1</color></ambient>
            <diffuse><texture texture="grass_sampler" texcoord="UVMap"/></diffuse>
            <specular><color>0 0 0 1</color></specular>
            <shininess><float>0.05</float></shininess>
          </phong>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="grass_material" name="grass_material">
      <instance_effect url="#grass_effect"/>
    </material>
  </library_materials>
  <library_geometries>
    <geometry id="grass_plane-mesh" name="grass_plane">
      <mesh>
        <source id="grass_plane-positions">
          <float_array id="grass_plane-positions-array" count="12">-0.5 -0.5 0 0.5 -0.5 0 0.5 0.5 0 -0.5 0.5 0</float_array>
          <technique_common>
            <accessor source="#grass_plane-positions-array" count="4" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="grass_plane-normals">
          <float_array id="grass_plane-normals-array" count="3">0 0 1</float_array>
          <technique_common>
            <accessor source="#grass_plane-normals-array" count="1" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="grass_plane-map-0">
          <float_array id="grass_plane-map-0-array" count="8">)"
      << uv.str() << R"(</float_array>
          <technique_common>
            <accessor source="#grass_plane-map-0-array" count="4" stride="2">
              <param name="S" type="float"/><param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="grass_plane-vertices">
          <input semantic="POSITION" source="#grass_plane-positions"/>
        </vertices>
        <triangles count="2" material="grass_material">
          <input semantic="VERTEX" source="#grass_plane-vertices" offset="0"/>
          <input semantic="NORMAL" source="#grass_plane-normals" offset="1"/>
          <input semantic="TEXCOORD" source="#grass_plane-map-0" offset="2" set="0"/>
          <p>0 0 0 1 0 1 2 0 2 0 0 0 2 0 2 3 0 3</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="grass_plane-node" name="grass_plane" type="NODE">
        <instance_geometry url="#grass_plane-mesh">
          <bind_material>
            <technique_common>
              <instance_material symbol="grass_material" target="#grass_material">
                <bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/>
              </instance_material>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
)";

  std::ofstream out(file_path);
  if (!out) {
    // Read-only install: fall back to the shipped plane so the ground still
    // renders, only with the texture scaled to the plane instead of to metres.
    RCLCPP_WARN(this->get_logger(),
                "Could not write '%s'; falling back to the fixed-scale grass plane.",
                file_path.c_str());
    return fallback;
  }
  out << dae.str();
  return resource;
}

void RosOutputAdapter::fit_ground_plane_to_track(
    visualization_msgs::msg::Marker& ground) const {
  const auto& cones = map_snapshot_cache_.ground_truth;
  if (cones.empty()) {
    return;
  }
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = min_x;
  double max_y = max_x;
  for (const auto& cone : cones) {
    min_x = std::min(min_x, cone.position.x);
    max_x = std::max(max_x, cone.position.x);
    min_y = std::min(min_y, cone.position.y);
    max_y = std::max(max_y, cone.position.y);
  }
  ground.pose.position.x = 0.5 * (min_x + max_x);
  ground.pose.position.y = 0.5 * (min_y + max_y);
  ground.scale.x = (max_x - min_x) + 2.0 * ground_margin_;
  ground.scale.y = (max_y - min_y) + 2.0 * ground_margin_;

  if (use_generated_track_visualization_) {
    if (grass_mesh_resource_.empty()) {
      grass_mesh_resource_ = ensure_grass_plane_mesh(ground.scale.x, ground.scale.y);
    }
    ground.mesh_resource = grass_mesh_resource_;
  }
}

void RosOutputAdapter::publish_visualization_ground(const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray ground_marker_array = ground_marker_template_;
  for (auto& marker : ground_marker_array.markers) {
    marker.header.stamp = stamp;
  }
  if (!ground_marker_array.markers.empty()) {
    fit_ground_plane_to_track(ground_marker_array.markers.front());
  }
  if (use_generated_track_visualization_) {
    add_track_surface_markers(ground_marker_array, map_snapshot_cache_.ground_truth, stamp);
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
    cone_hit_match_distance_ = collision["hit_match_distance"].as<double>(cone_hit_match_distance_);
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
  double color_a = 1.0;
  double timing_line_target_cell_length = 0.5;
  int timing_line_row_count = 2;
  double timing_line_total_width = 0.45;
  double timing_line_z = 0.01;
  double timing_line_height = 0.02;

  const YAML::Node visualization = config["visualization"];
  if (visualization) {
    const std::string track_visualization =
        visualization["track_visualization"].as<std::string>("generated_track");
    use_generated_track_visualization_ = track_visualization != "asphalt_texture_everywhere";

    const YAML::Node track_surface = visualization["track_surface"];
    if (track_surface) {
      track_surface_max_boundary_segment_length_ =
          track_surface["max_boundary_segment_length"].as<double>(
              track_surface_max_boundary_segment_length_);
      track_surface_max_loop_segment_length_ = track_surface["max_loop_segment_length"].as<double>(
          track_surface_max_loop_segment_length_);
      track_surface_cone_clearance_ =
          track_surface["cone_clearance"].as<double>(track_surface_cone_clearance_);
      track_surface_curb_width_ = track_surface["curb_width"].as<double>(track_surface_curb_width_);
      track_surface_curb_block_length_ =
          track_surface["curb_block_length"].as<double>(track_surface_curb_block_length_);
      track_surface_curb_z_ = track_surface["curb_z"].as<double>(track_surface_curb_z_);
      track_surface_asphalt_z_ = track_surface["asphalt_z"].as<double>(track_surface_asphalt_z_);
    }

    const YAML::Node position = visualization["position"];
    if (position) {
      ground_z_ = position["z"].as<double>(ground_z_);
    }
    ground_margin_ = visualization["margin"].as<double>(ground_margin_);
    ground_fallback_size_ = visualization["fallback_size"].as<double>(ground_fallback_size_);
    grass_tile_size_m_ = visualization["grass_tile_size_m"].as<double>(grass_tile_size_m_);

    const YAML::Node color = visualization["color"];
    if (color) {
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
  ground.pose.position.x = 0.0;
  ground.pose.position.y = 0.0;
  ground.pose.position.z = ground_z_;
  ground.pose.orientation.w = 1.0;
  ground.scale.x = ground_fallback_size_;
  ground.scale.y = ground_fallback_size_;
  ground.scale.z = 1.0;
  ground.color.r = 1.0f;
  ground.color.g = 1.0f;
  ground.color.b = 1.0f;
  ground.color.a = static_cast<float>(color_a);
  ground.mesh_resource = use_generated_track_visualization_
                             ? "package://invictasim/resources/meshes/ground/grass_plane.dae"
                             : "package://invictasim/resources/meshes/ground/asphalt_plane.dae";
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
  body_marker_template_.color.r = 0.82f;
  body_marker_template_.color.g = 0.02f;
  body_marker_template_.color.b = 0.03f;
  body_marker_template_.mesh_resource = get_car_mesh_resource("car_body.glb");
  body_marker_template_.mesh_use_embedded_materials = car_folder != "02";

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

std_msgs::msg::ColorRGBA RosOutputAdapter::make_track_color(float r, float g, float b) const {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = 1.0f;
  return color;
}

geometry_msgs::msg::Point RosOutputAdapter::make_track_point(double x, double y) const {
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

std::array<double, 2> RosOutputAdapter::track_segment_outward_normal(
    const std::vector<common_lib::structures::Cone>& side_cones,
    const std::vector<common_lib::structures::Cone>& opposite_cones, std::size_t start_i,
    std::size_t end_i) const {
  const auto& start = side_cones[start_i];
  const auto& end = side_cones[end_i];
  const double dx = end.position.x - start.position.x;
  const double dy = end.position.y - start.position.y;
  const double length = std::hypot(dx, dy);
  if (length <= std::numeric_limits<double>::epsilon()) {
    return {0.0, 0.0};
  }

  std::array<double, 2> normal{-dy / length, dx / length};
  const double mid_x = 0.5 * (start.position.x + end.position.x);
  const double mid_y = 0.5 * (start.position.y + end.position.y);
  double closest_distance_squared = std::numeric_limits<double>::max();
  std::size_t closest_opposite_i = 0;
  for (std::size_t i = 0; i < opposite_cones.size(); ++i) {
    const double x = opposite_cones[i].position.x - mid_x;
    const double y = opposite_cones[i].position.y - mid_y;
    const double distance_squared = x * x + y * y;
    if (distance_squared < closest_distance_squared) {
      closest_distance_squared = distance_squared;
      closest_opposite_i = i;
    }
  }

  const double to_opposite_x = opposite_cones[closest_opposite_i].position.x - mid_x;
  const double to_opposite_y = opposite_cones[closest_opposite_i].position.y - mid_y;
  if (normal[0] * to_opposite_x + normal[1] * to_opposite_y > 0.0) {
    normal[0] = -normal[0];
    normal[1] = -normal[1];
  }
  return normal;
}

void RosOutputAdapter::add_track_quad(visualization_msgs::msg::Marker& marker,
                                      const geometry_msgs::msg::Point& a,
                                      const geometry_msgs::msg::Point& b,
                                      const geometry_msgs::msg::Point& c,
                                      const geometry_msgs::msg::Point& d,
                                      const std_msgs::msg::ColorRGBA& color) const {
  const std::array<const geometry_msgs::msg::Point*, 6> corners{&a, &b, &c, &a, &c, &d};
  for (const auto* corner : corners) {
    marker.points.push_back(*corner);
    marker.colors.push_back(color);
  }
}

visualization_msgs::msg::Marker RosOutputAdapter::make_track_triangle_marker(
    const std::string& ns, int id, double z, const rclcpp::Time& stamp) const {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.z = z;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  return marker;
}

RosOutputAdapter::TrackRibbon RosOutputAdapter::build_track_ribbon(
    const std::vector<common_lib::structures::Cone>& side_cones,
    const std::vector<common_lib::structures::Cone>& opposite_cones, bool closed) const {
  TrackRibbon ribbon;
  ribbon.inner.resize(side_cones.size());
  ribbon.outer.resize(side_cones.size());
  ribbon.closed = closed;

  for (std::size_t i = 0; i < side_cones.size(); ++i) {
    const bool has_previous = ribbon.closed || i > 0;
    const bool has_next = ribbon.closed || i + 1 < side_cones.size();
    std::array<double, 2> normal{0.0, 0.0};
    if (has_previous) {
      const std::size_t previous_i = i == 0 ? side_cones.size() - 1 : i - 1;
      const auto previous_normal =
          track_segment_outward_normal(side_cones, opposite_cones, previous_i, i);
      normal[0] += previous_normal[0];
      normal[1] += previous_normal[1];
    }
    if (has_next) {
      const std::size_t next_i = i + 1 == side_cones.size() ? 0 : i + 1;
      const auto next_normal = track_segment_outward_normal(side_cones, opposite_cones, i, next_i);
      normal[0] += next_normal[0];
      normal[1] += next_normal[1];
    }

    const double normal_length = std::hypot(normal[0], normal[1]);
    if (normal_length <= std::numeric_limits<double>::epsilon()) {
      continue;
    }
    normal[0] /= normal_length;
    normal[1] /= normal_length;

    const auto& cone = side_cones[i];
    ribbon.inner[i] = make_track_point(cone.position.x + normal[0] * track_surface_cone_clearance_,
                                       cone.position.y + normal[1] * track_surface_cone_clearance_);
    ribbon.outer[i] = make_track_point(
        cone.position.x + normal[0] * (track_surface_cone_clearance_ + track_surface_curb_width_),
        cone.position.y + normal[1] * (track_surface_cone_clearance_ + track_surface_curb_width_));
  }
  return ribbon;
}

double RosOutputAdapter::track_path_length(const std::vector<geometry_msgs::msg::Point>& points,
                                           bool closed) const {
  double length = 0.0;
  const std::size_t segment_count = closed ? points.size() : points.size() - 1;
  for (std::size_t i = 0; i < segment_count; ++i) {
    const std::size_t next_i = (i + 1) % points.size();
    length += std::hypot(points[next_i].x - points[i].x, points[next_i].y - points[i].y);
  }
  return length;
}

geometry_msgs::msg::Point RosOutputAdapter::sample_track_path(
    const std::vector<geometry_msgs::msg::Point>& points, bool closed,
    double target_distance) const {
  const std::size_t segment_count = closed ? points.size() : points.size() - 1;
  double distance = std::max(0.0, target_distance);
  for (std::size_t i = 0; i < segment_count; ++i) {
    const std::size_t next_i = (i + 1) % points.size();
    const double segment_length =
        std::hypot(points[next_i].x - points[i].x, points[next_i].y - points[i].y);
    if (segment_length <= std::numeric_limits<double>::epsilon()) {
      continue;
    }
    if (distance <= segment_length || i + 1 == segment_count) {
      const double t = std::clamp(distance / segment_length, 0.0, 1.0);
      return make_track_point(points[i].x + (points[next_i].x - points[i].x) * t,
                              points[i].y + (points[next_i].y - points[i].y) * t);
    }
    distance -= segment_length;
  }
  return points.back();
}

geometry_msgs::msg::Point RosOutputAdapter::closest_point_on_track_path(
    const std::vector<geometry_msgs::msg::Point>& points, bool closed,
    const geometry_msgs::msg::Point& query) const {
  geometry_msgs::msg::Point closest = points.front();
  double closest_distance_squared = std::numeric_limits<double>::max();
  const std::size_t segment_count = closed ? points.size() : points.size() - 1;
  for (std::size_t i = 0; i < segment_count; ++i) {
    const std::size_t next_i = (i + 1) % points.size();
    const double dx = points[next_i].x - points[i].x;
    const double dy = points[next_i].y - points[i].y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared <= std::numeric_limits<double>::epsilon()) {
      continue;
    }

    const double query_dx = query.x - points[i].x;
    const double query_dy = query.y - points[i].y;
    const double t = std::clamp((query_dx * dx + query_dy * dy) / length_squared, 0.0, 1.0);
    const geometry_msgs::msg::Point candidate =
        make_track_point(points[i].x + dx * t, points[i].y + dy * t);
    const double distance_x = candidate.x - query.x;
    const double distance_y = candidate.y - query.y;
    const double distance_squared = distance_x * distance_x + distance_y * distance_y;
    if (distance_squared < closest_distance_squared) {
      closest = candidate;
      closest_distance_squared = distance_squared;
    }
  }
  return closest;
}

void RosOutputAdapter::add_track_curb_ribbon(visualization_msgs::msg::Marker& curb_marker,
                                             const TrackRibbon& ribbon,
                                             const std_msgs::msg::ColorRGBA& curb_red,
                                             const std_msgs::msg::ColorRGBA& curb_white) const {
  int color_index = 0;
  const std::size_t segment_count = ribbon.closed ? ribbon.inner.size() : ribbon.inner.size() - 1;
  for (std::size_t i = 0; i < segment_count; ++i) {
    const std::size_t next_i = (i + 1) % ribbon.inner.size();
    const double length = std::hypot(ribbon.inner[next_i].x - ribbon.inner[i].x,
                                     ribbon.inner[next_i].y - ribbon.inner[i].y);
    const double max_segment_length = next_i == 0 ? track_surface_max_loop_segment_length_
                                                  : track_surface_max_boundary_segment_length_;
    if (length <= std::numeric_limits<double>::epsilon() || length > max_segment_length) {
      continue;
    }

    const int block_count =
        std::max(1, static_cast<int>(std::ceil(length / track_surface_curb_block_length_)));
    for (int block = 0; block < block_count; ++block) {
      const double t0 = static_cast<double>(block) / static_cast<double>(block_count);
      const double t1 = static_cast<double>(block + 1) / static_cast<double>(block_count);
      const auto inner_a =
          make_track_point(ribbon.inner[i].x + (ribbon.inner[next_i].x - ribbon.inner[i].x) * t0,
                           ribbon.inner[i].y + (ribbon.inner[next_i].y - ribbon.inner[i].y) * t0);
      const auto inner_b =
          make_track_point(ribbon.inner[i].x + (ribbon.inner[next_i].x - ribbon.inner[i].x) * t1,
                           ribbon.inner[i].y + (ribbon.inner[next_i].y - ribbon.inner[i].y) * t1);
      const auto outer_a =
          make_track_point(ribbon.outer[i].x + (ribbon.outer[next_i].x - ribbon.outer[i].x) * t0,
                           ribbon.outer[i].y + (ribbon.outer[next_i].y - ribbon.outer[i].y) * t0);
      const auto outer_b =
          make_track_point(ribbon.outer[i].x + (ribbon.outer[next_i].x - ribbon.outer[i].x) * t1,
                           ribbon.outer[i].y + (ribbon.outer[next_i].y - ribbon.outer[i].y) * t1);
      add_track_quad(curb_marker, inner_a, inner_b, outer_b, outer_a,
                     (color_index % 2) == 0 ? curb_red : curb_white);
      ++color_index;
    }
  }
}

void RosOutputAdapter::add_track_asphalt_from_ribbon(
    visualization_msgs::msg::Marker& asphalt_marker, const TrackRibbon& source_ribbon,
    const TrackRibbon& target_ribbon, const std_msgs::msg::ColorRGBA& asphalt_color,
    bool reverse_winding) const {
  constexpr double asphalt_sample_length = 2.0;
  const double source_length = track_path_length(source_ribbon.inner, source_ribbon.closed);
  const int segment_count =
      std::max(1, static_cast<int>(std::ceil(source_length / asphalt_sample_length)));
  std::vector<AsphaltSample> samples;
  samples.reserve(static_cast<std::size_t>(segment_count) + 1);
  std::vector<double> widths;
  widths.reserve(static_cast<std::size_t>(segment_count) + 1);
  for (int sample = 0; sample <= segment_count; ++sample) {
    const double distance =
        source_length * static_cast<double>(sample) / static_cast<double>(segment_count);
    AsphaltSample asphalt_sample;
    asphalt_sample.source = sample_track_path(source_ribbon.inner, source_ribbon.closed, distance);
    asphalt_sample.target = closest_point_on_track_path(target_ribbon.inner, target_ribbon.closed,
                                                        asphalt_sample.source);
    asphalt_sample.width = std::hypot(asphalt_sample.target.x - asphalt_sample.source.x,
                                      asphalt_sample.target.y - asphalt_sample.source.y);
    widths.push_back(asphalt_sample.width);
    samples.push_back(asphalt_sample);
  }

  std::nth_element(widths.begin(), widths.begin() + static_cast<long>(widths.size() / 2),
                   widths.end());
  const double max_width = widths[widths.size() / 2] * 1.5;
  for (int segment = 0; segment < segment_count; ++segment) {
    const auto& start = samples[static_cast<std::size_t>(segment)];
    const auto& end = samples[static_cast<std::size_t>(segment + 1)];
    const double target_step =
        std::hypot(end.target.x - start.target.x, end.target.y - start.target.y);
    if (start.width > max_width || end.width > max_width ||
        target_step > track_surface_max_boundary_segment_length_) {
      continue;
    }

    if (reverse_winding) {
      add_track_quad(asphalt_marker, start.target, end.target, end.source, start.source,
                     asphalt_color);
    } else {
      add_track_quad(asphalt_marker, start.source, end.source, end.target, start.target,
                     asphalt_color);
    }
  }
}

RosOutputAdapter::TrackLayout RosOutputAdapter::track_layout() const {
  std::string discipline = simulator_->get_discipline();
  std::transform(discipline.begin(), discipline.end(), discipline.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (discipline == "acceleration") {
    return TrackLayout::kOpenLanes;
  }
  if (discipline == "skidpad") {
    return TrackLayout::kSkidpad;
  }
  return TrackLayout::kClosedLoop;
}

std::vector<std::vector<common_lib::structures::Cone>> RosOutputAdapter::split_cone_runs(
    const std::vector<common_lib::structures::Cone>& cones) const {
  if (cones.size() < 3) {
    return {cones};
  }
  
  std::vector<double> spacing;
  spacing.reserve(cones.size() - 1);
  for (std::size_t i = 0; i + 1 < cones.size(); ++i) {
    spacing.push_back(cones[i].position.euclidean_distance(cones[i + 1].position));
  }
  std::vector<double> sorted_spacing = spacing;
  std::nth_element(sorted_spacing.begin(), sorted_spacing.begin() + sorted_spacing.size() / 2,
                   sorted_spacing.end());
  const double median = sorted_spacing[sorted_spacing.size() / 2];
  const double break_threshold = std::max(2.5 * median, 1e-3);

  std::vector<std::vector<common_lib::structures::Cone>> runs;
  runs.emplace_back();
  for (std::size_t i = 0; i < cones.size(); ++i) {
    if (i > 0 && spacing[i - 1] > break_threshold) {
      runs.emplace_back();
    }
    runs.back().push_back(cones[i]);
  }
  return runs;
}

void RosOutputAdapter::add_track_surface_markers(
    visualization_msgs::msg::MarkerArray& marker_array,
    const std::vector<common_lib::structures::Cone>& cones, const rclcpp::Time& stamp) const {
  std::vector<common_lib::structures::Cone> blue_cones;
  std::vector<common_lib::structures::Cone> yellow_cones;
  for (const auto& cone : cones) {
    if (cone.color == common_lib::competition_logic::Color::BLUE) {
      blue_cones.push_back(cone);
    } else if (cone.color == common_lib::competition_logic::Color::YELLOW) {
      yellow_cones.push_back(cone);
    }
  }

  if (blue_cones.size() < 2 || yellow_cones.size() < 2) {
    return;
  }

  const auto curb_red = make_track_color(0.88f, 0.02f, 0.02f);
  const auto curb_white = make_track_color(0.96f, 0.96f, 0.92f);
  const auto asphalt_color = make_track_color(0.18f, 0.18f, 0.17f);
  auto asphalt_marker =
      make_track_triangle_marker("invictasim_track_asphalt", 1000, track_surface_asphalt_z_, stamp);
  auto curb_marker =
      make_track_triangle_marker("invictasim_track_curbs", 3000, track_surface_curb_z_, stamp);

  const TrackLayout layout = track_layout();
  const auto blue_runs = split_cone_runs(blue_cones);
  const auto yellow_runs = split_cone_runs(yellow_cones);

  const auto centroid = [](const std::vector<common_lib::structures::Cone>& run) {
    double x = 0.0;
    double y = 0.0;
    for (const auto& cone : run) {
      x += cone.position.x;
      y += cone.position.y;
    }
    return std::array<double, 2>{x / static_cast<double>(run.size()),
                                 y / static_cast<double>(run.size())};
  };

  // A run is only joined end to end when the event actually has a closed
  // boundary and the run really does come back on itself. Acceleration never
  // closes; on a skidpad each circle closes independently, which is why the
  // whole colour cannot be tested as one boundary.
  const auto run_is_closed = [&](const std::vector<common_lib::structures::Cone>& run) {
    if (layout == TrackLayout::kOpenLanes || run.size() < 3) {
      return false;
    }
    return run.back().position.euclidean_distance(run.front().position) <=
           track_surface_max_loop_segment_length_;
  };

  for (const auto& blue_run : blue_runs) {
    if (blue_run.size() < 2) {
      continue;
    }
    // Pair each run with the opposite-colour run that shares its piece of track.
    // Index order cannot be used: on a skidpad the outer arc of one circle is
    // stored alongside the inner arc of the other.
    const std::vector<common_lib::structures::Cone>* partner = &yellow_runs.front();
    if (yellow_runs.size() > 1) {
      const auto blue_centre = centroid(blue_run);
      double best = std::numeric_limits<double>::max();
      for (const auto& yellow_run : yellow_runs) {
        if (yellow_run.size() < 2) {
          continue;
        }
        const auto yellow_centre = centroid(yellow_run);
        const double distance =
            std::hypot(blue_centre[0] - yellow_centre[0], blue_centre[1] - yellow_centre[1]);
        if (distance < best) {
          best = distance;
          partner = &yellow_run;
        }
      }
    }

    const auto blue_ribbon = build_track_ribbon(blue_run, *partner, run_is_closed(blue_run));
    const auto yellow_ribbon = build_track_ribbon(*partner, blue_run, run_is_closed(*partner));
    add_track_asphalt_from_ribbon(asphalt_marker, blue_ribbon, yellow_ribbon, asphalt_color, false);
    add_track_asphalt_from_ribbon(asphalt_marker, yellow_ribbon, blue_ribbon, asphalt_color, true);
    add_track_curb_ribbon(curb_marker, blue_ribbon, curb_red, curb_white);
    add_track_curb_ribbon(curb_marker, yellow_ribbon, curb_red, curb_white);
  }

  if (!asphalt_marker.points.empty()) {
    marker_array.markers.push_back(asphalt_marker);
  }
  if (!curb_marker.points.empty()) {
    marker_array.markers.push_back(curb_marker);
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
