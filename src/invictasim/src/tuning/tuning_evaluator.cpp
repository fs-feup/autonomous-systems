#include "tuning/tuning_evaluator.hpp"

#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace {
constexpr const char* kDefaultEvaluationOutputDirectory = "performance/invictasim_evaluation";
constexpr int kEvaluatorFrequencyHz = 50;
}  // namespace

double TuningEvaluator::RunningRmse::update(double value) {
  sum_squares += value * value;
  count += 1;
  return std::sqrt(sum_squares / static_cast<double>(count));
}

double TuningEvaluator::normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

TuningEvaluator::PoseSample TuningEvaluator::transform_pose_to_map(
    const PoseSample& pose, const PoseSample& source_origin, const PoseSample& target_origin) {
  const double dx = pose.x - source_origin.x;
  const double dy = pose.y - source_origin.y;
  const double source_cos = std::cos(source_origin.yaw);
  const double source_sin = std::sin(source_origin.yaw);
  const double local_x = source_cos * dx + source_sin * dy;
  const double local_y = -source_sin * dx + source_cos * dy;
  const double local_yaw = normalize_angle(pose.yaw - source_origin.yaw);

  const double target_cos = std::cos(target_origin.yaw);
  const double target_sin = std::sin(target_origin.yaw);
  return {target_origin.x + target_cos * local_x - target_sin * local_y,
          target_origin.y + target_sin * local_x + target_cos * local_y,
          normalize_angle(target_origin.yaw + local_yaw)};
}

TuningEvaluator::TuningEvaluator(const std::shared_ptr<InvictaSim>& simulator,
                                 const InvictaSimParameters& /*params*/)
    : Node("invictasim_tuning_evaluator"), simulator_(simulator) {
  output_directory_ = this->declare_parameter<std::string>(
      "tuning_output_directory", kDefaultEvaluationOutputDirectory);
  output_samples_ = this->declare_parameter<bool>("tuning_output_samples", false);

  position_error_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/tuning/position_error", 10);
  position_rmse_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/tuning/position_rmse", 10);
  heading_error_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/tuning/heading_error", 10);
  heading_rmse_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/tuning/heading_rmse", 10);
  velocity_error_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/tuning/velocity_error", 10);
  velocity_rmse_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/tuning/velocity_rmse", 10);
  yaw_rate_error_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/tuning/yaw_rate_error", 10);
  yaw_rate_rmse_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/tuning/yaw_rate_rmse", 10);
  front_wheel_rpm_error_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/tuning/front_wheel_rpm_error",
                                                     10);
  front_wheel_rpm_rmse_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/tuning/front_wheel_rpm_rmse",
                                                     10);
  motor_rpm_error_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/tuning/motor_rpm_error", 10);
  motor_rpm_rmse_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/tuning/motor_rpm_rmse", 10);
  real_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/invictasim/tuning/real_pose", 10);
  sim_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/invictasim/tuning/sim_pose", 10);
  visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/invictasim/tuning/visualization", 10);

  control_sub_ = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      "/control/command", 50,
      [this](const custom_interfaces::msg::ControlCommand::SharedPtr msg) {
        control_callback(msg);
      });
  real_pose_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
      "/state_estimation/vehicle_pose", 100,
      [this](const custom_interfaces::msg::Pose::SharedPtr msg) { real_pose_callback(msg); });
  real_velocity_sub_ = this->create_subscription<custom_interfaces::msg::Velocities>(
      "/state_estimation/velocities", 100,
      [this](const custom_interfaces::msg::Velocities::SharedPtr msg) {
        real_velocity_callback(msg);
      });
  real_fl_rpm_sub_ = this->create_subscription<custom_interfaces::msg::WheelRPM>(
      "/vehicle/fl_rpm", 100,
      [this](const custom_interfaces::msg::WheelRPM::SharedPtr msg) { real_fl_rpm_callback(msg); });
  real_fr_rpm_sub_ = this->create_subscription<custom_interfaces::msg::WheelRPM>(
      "/vehicle/fr_rpm", 100,
      [this](const custom_interfaces::msg::WheelRPM::SharedPtr msg) { real_fr_rpm_callback(msg); });
  real_motor_rpm_sub_ = this->create_subscription<custom_interfaces::msg::WheelRPM>(
      "/vehicle/motor_rpm", 100,
      [this](const custom_interfaces::msg::WheelRPM::SharedPtr msg) {
        real_motor_rpm_callback(msg);
      });

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / kEvaluatorFrequencyHz),
                                   [this]() { on_timer(); });

  RCLCPP_INFO(this->get_logger(),
              "Tuning evaluator enabled inside InvictaSim. Waiting for /control/command.");
}

TuningEvaluator::~TuningEvaluator() {
  if (!output_samples_) {
    write_summary_csv();
  }
  if (csv_file_.is_open()) {
    csv_file_.close();
  }
}

void TuningEvaluator::control_callback(
    const custom_interfaces::msg::ControlCommand::SharedPtr /*msg*/) {
  has_control_command_ = true;
  try_start_evaluation();
}

void TuningEvaluator::try_start_evaluation() {
  if (started_) {
    return;
  }
  if (!has_control_command_) {
    return;
  }
  if (!has_real_pose_) {
    RCLCPP_WARN(this->get_logger(),
                "Received control before real pose stream; waiting for the first real pose.");
    return;
  }

  const VehicleStateSnapshot sim_state = simulator_->get_vehicle_state_snapshot();
  real_origin_ = {latest_real_pose_.x, latest_real_pose_.y, latest_real_pose_.theta};
  sim_origin_ = {sim_state.position.x, sim_state.position.y, sim_state.yaw};
  start_time_ = std::chrono::steady_clock::now();
  started_ = true;
  open_csv();
  RCLCPP_INFO(this->get_logger(), "Tuning evaluation started and trajectories aligned.");
}

void TuningEvaluator::real_pose_callback(const custom_interfaces::msg::Pose::SharedPtr msg) {
  latest_real_pose_ = *msg;
  has_real_pose_ = true;
  try_start_evaluation();
}

void TuningEvaluator::real_velocity_callback(
    const custom_interfaces::msg::Velocities::SharedPtr msg) {
  latest_real_velocity_ = *msg;
  has_real_velocity_ = true;
}

void TuningEvaluator::real_fl_rpm_callback(
    const custom_interfaces::msg::WheelRPM::SharedPtr msg) {
  latest_real_fl_rpm_ = msg->fl_rpm;
  has_real_fl_rpm_ = true;
}

void TuningEvaluator::real_fr_rpm_callback(
    const custom_interfaces::msg::WheelRPM::SharedPtr msg) {
  latest_real_fr_rpm_ = msg->fr_rpm;
  has_real_fr_rpm_ = true;
}

void TuningEvaluator::real_motor_rpm_callback(
    const custom_interfaces::msg::WheelRPM::SharedPtr msg) {
  latest_real_motor_rpm_ = msg->rr_rpm;
  has_real_motor_rpm_ = true;
}

void TuningEvaluator::on_timer() {
  if (!started_ || !has_real_pose_) {
    return;
  }

  const VehicleStateSnapshot sim_state = simulator_->get_vehicle_state_snapshot();
  const SensorsSnapshot sim_sensors = simulator_->get_sensors_snapshot();
  const PoseSample real_raw = {latest_real_pose_.x, latest_real_pose_.y, latest_real_pose_.theta};
  const PoseSample real_pose = transform_pose_to_map(real_raw, real_origin_, sim_origin_);
  const PoseSample sim_pose = {sim_state.position.x, sim_state.position.y, sim_state.yaw};

  const double position_error = std::hypot(sim_pose.x - real_pose.x, sim_pose.y - real_pose.y);
  const double heading_error = normalize_angle(sim_pose.yaw - real_pose.yaw);
  publish_float(position_error_pub_, position_error);
  publish_float(position_rmse_pub_, position_rmse_.update(position_error));
  publish_float(heading_error_pub_, heading_error);
  publish_float(heading_rmse_pub_, heading_rmse_.update(heading_error));

  if (has_real_velocity_) {
    const VelocitySample real_velocity = {latest_real_velocity_.velocity_x,
                                          latest_real_velocity_.velocity_y,
                                          latest_real_velocity_.angular_velocity};
    const VelocitySample sim_velocity = {sim_state.velocity_x, sim_state.velocity_y,
                                         sim_state.yaw_rate};
    const double velocity_error =
        std::hypot(sim_velocity.velocity_x - real_velocity.velocity_x,
                   sim_velocity.velocity_y - real_velocity.velocity_y);
    const double yaw_rate_error = sim_velocity.yaw_rate - real_velocity.yaw_rate;
    publish_float(velocity_error_pub_, velocity_error);
    publish_float(velocity_rmse_pub_, velocity_rmse_.update(velocity_error));
    publish_float(yaw_rate_error_pub_, yaw_rate_error);
    publish_float(yaw_rate_rmse_pub_, yaw_rate_rmse_.update(yaw_rate_error));
  }

  if (has_real_fl_rpm_ && has_real_fr_rpm_) {
    const double real_front_rpm = 0.5 * (latest_real_fl_rpm_ + latest_real_fr_rpm_);
    const double sim_front_rpm =
        0.5 * (sim_sensors.wheel_rpm.front_left + sim_sensors.wheel_rpm.front_right);
    const double front_wheel_rpm_error = sim_front_rpm - real_front_rpm;
    publish_float(front_wheel_rpm_error_pub_, front_wheel_rpm_error);
    publish_float(front_wheel_rpm_rmse_pub_, front_wheel_rpm_rmse_.update(front_wheel_rpm_error));
  }

  if (has_real_motor_rpm_) {
    const double motor_rpm_error = sim_sensors.motor_rpm - latest_real_motor_rpm_;
    publish_float(motor_rpm_error_pub_, motor_rpm_error);
    publish_float(motor_rpm_rmse_pub_, motor_rpm_rmse_.update(motor_rpm_error));
  }

  publish_pose(real_pose, true);
  publish_pose(sim_pose, false);
  publish_comparison_visualization(real_pose, sim_pose, position_error);
  write_csv_row(real_pose, sim_pose, position_error, heading_error);
}

void TuningEvaluator::publish_pose(const PoseSample& pose, bool real) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = this->now();
  pose_stamped.header.frame_id = "map";
  pose_stamped.pose.position.x = pose.x;
  pose_stamped.pose.position.y = pose.y;
  pose_stamped.pose.orientation.z = std::sin(pose.yaw / 2.0);
  pose_stamped.pose.orientation.w = std::cos(pose.yaw / 2.0);

  if (real) {
    real_pose_pub_->publish(pose_stamped);
  } else {
    sim_pose_pub_->publish(pose_stamped);
  }
}

void TuningEvaluator::publish_float(
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& publisher, double value) {
  std_msgs::msg::Float64 msg;
  msg.data = value;
  publisher->publish(msg);
}

void TuningEvaluator::publish_comparison_visualization(const PoseSample& real_pose,
                                                       const PoseSample& sim_pose,
                                                       double position_error) {
  const builtin_interfaces::msg::Time stamp = this->now();
  visualization_msgs::msg::Marker real_car;
  real_car.header.stamp = stamp;
  real_car.header.frame_id = "map";
  real_car.ns = "tuning_real_vehicle";
  real_car.id = 0;
  real_car.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  real_car.action = visualization_msgs::msg::Marker::ADD;
  real_car.pose.position.x = real_pose.x;
  real_car.pose.position.y = real_pose.y;
  real_car.pose.orientation.z = std::sin(real_pose.yaw / 2.0);
  real_car.pose.orientation.w = std::cos(real_pose.yaw / 2.0);
  real_car.scale.x = 1.0;
  real_car.scale.y = 1.0;
  real_car.scale.z = 1.0;
  real_car.color.g = 0.75;
  real_car.color.b = 1.0;
  real_car.color.a = 0.85;
  real_car.mesh_resource = "package://invictasim/resources/meshes/car/02/car_body.glb";
  real_car.mesh_use_embedded_materials = false;

  visualization_msgs::msg::Marker error_line;
  error_line.header.stamp = stamp;
  error_line.header.frame_id = "map";
  error_line.ns = "tuning_position_error";
  error_line.id = 1;
  error_line.type = visualization_msgs::msg::Marker::LINE_LIST;
  error_line.action = visualization_msgs::msg::Marker::ADD;
  error_line.pose.orientation.w = 1.0;
  error_line.scale.x = 0.06;
  error_line.color.r = 1.0;
  error_line.color.g = 0.1;
  error_line.color.b = 0.1;
  error_line.color.a = 1.0;
  geometry_msgs::msg::Point real_point;
  real_point.x = real_pose.x;
  real_point.y = real_pose.y;
  real_point.z = 0.4;
  geometry_msgs::msg::Point sim_point;
  sim_point.x = sim_pose.x;
  sim_point.y = sim_pose.y;
  sim_point.z = 0.4;
  error_line.points = {real_point, sim_point};

  visualization_msgs::msg::Marker distance_text;
  distance_text.header.stamp = stamp;
  distance_text.header.frame_id = "map";
  distance_text.ns = "tuning_position_error";
  distance_text.id = 2;
  distance_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  distance_text.action = visualization_msgs::msg::Marker::ADD;
  distance_text.pose.position.x = 0.5 * (real_pose.x + sim_pose.x);
  distance_text.pose.position.y = 0.5 * (real_pose.y + sim_pose.y);
  distance_text.pose.position.z = 1.2;
  distance_text.pose.orientation.w = 1.0;
  distance_text.scale.z = 0.45;
  distance_text.color.r = 1.0;
  distance_text.color.g = 1.0;
  distance_text.color.b = 1.0;
  distance_text.color.a = 1.0;
  std::ostringstream text;
  text << std::fixed << std::setprecision(2) << position_error << " m";
  distance_text.text = text.str();

  visualization_msgs::msg::MarkerArray markers;
  markers.markers = {real_car, error_line, distance_text};
  visualization_pub_->publish(markers);
}

void TuningEvaluator::open_csv() {
  std::filesystem::create_directories(output_directory_);
  if (!output_samples_) {
    return;
  }
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&now_time, &local_time);
  std::ostringstream filename;
  filename << "tuning_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".csv";
  const std::filesystem::path csv_path = output_directory_ / filename.str();
  csv_file_.open(csv_path);
  csv_file_
      << "elapsed_s,real_x,real_y,real_yaw,sim_x,sim_y,sim_yaw,position_error,heading_error,"
      << "real_velocity_x,real_velocity_y,real_yaw_rate,sim_velocity_x,sim_velocity_y,"
      << "sim_yaw_rate,real_fl_rpm,real_fr_rpm,real_front_rpm,real_motor_rpm,sim_fl_rpm,"
      << "sim_fr_rpm,sim_front_rpm,sim_motor_rpm,front_wheel_rpm_error,motor_rpm_error\n";
  RCLCPP_INFO(this->get_logger(), "Writing tuning samples to %s", csv_path.c_str());
}

void TuningEvaluator::write_summary_csv() {
  if (!started_) {
    return;
  }
  std::filesystem::create_directories(output_directory_);
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&now_time, &local_time);
  std::ostringstream filename;
  filename << "summary_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".csv";
  const std::filesystem::path summary_path = output_directory_ / filename.str();
  std::ofstream summary_file(summary_path);
  summary_file << "samples,position_rmse,heading_rmse,velocity_rmse,yaw_rate_rmse,"
               << "front_wheel_rpm_rmse,motor_rpm_rmse\n";
  const auto rmse_or_empty = [](const RunningRmse& metric) -> std::string {
    if (metric.count <= 0) {
      return "";
    }
    return std::to_string(std::sqrt(metric.sum_squares / static_cast<double>(metric.count)));
  };
  summary_file << position_rmse_.count << "," << rmse_or_empty(position_rmse_) << ","
               << rmse_or_empty(heading_rmse_) << "," << rmse_or_empty(velocity_rmse_) << ","
               << rmse_or_empty(yaw_rate_rmse_) << "," << rmse_or_empty(front_wheel_rpm_rmse_)
               << "," << rmse_or_empty(motor_rpm_rmse_) << "\n";
  RCLCPP_INFO(this->get_logger(), "Writing tuning summary to %s", summary_path.c_str());
}

void TuningEvaluator::write_csv_row(const PoseSample& real_pose, const PoseSample& sim_pose,
                                    double position_error, double heading_error) {
  if (!csv_file_.is_open()) {
    return;
  }
  const VehicleStateSnapshot sim_state = simulator_->get_vehicle_state_snapshot();
  const SensorsSnapshot sim_sensors = simulator_->get_sensors_snapshot();
  const double elapsed_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();

  const bool has_real_front_rpm = has_real_fl_rpm_ && has_real_fr_rpm_;
  const double real_front_rpm =
      has_real_front_rpm ? 0.5 * (latest_real_fl_rpm_ + latest_real_fr_rpm_) : 0.0;
  const double sim_front_rpm =
      0.5 * (sim_sensors.wheel_rpm.front_left + sim_sensors.wheel_rpm.front_right);

  csv_file_ << elapsed_s << "," << real_pose.x << "," << real_pose.y << "," << real_pose.yaw
            << "," << sim_pose.x << "," << sim_pose.y << "," << sim_pose.yaw << ","
            << position_error << "," << heading_error << ",";
  if (has_real_velocity_) {
    csv_file_ << latest_real_velocity_.velocity_x << "," << latest_real_velocity_.velocity_y << ","
              << latest_real_velocity_.angular_velocity << ",";
  } else {
    csv_file_ << ",,,";
  }
  csv_file_ << sim_state.velocity_x << "," << sim_state.velocity_y << "," << sim_state.yaw_rate
            << ",";

  csv_file_ << (has_real_fl_rpm_ ? std::to_string(latest_real_fl_rpm_) : "") << ","
            << (has_real_fr_rpm_ ? std::to_string(latest_real_fr_rpm_) : "") << ","
            << (has_real_front_rpm ? std::to_string(real_front_rpm) : "") << ","
            << (has_real_motor_rpm_ ? std::to_string(latest_real_motor_rpm_) : "") << ","
            << sim_sensors.wheel_rpm.front_left << "," << sim_sensors.wheel_rpm.front_right << ","
            << sim_front_rpm << "," << sim_sensors.motor_rpm << ",";

  if (has_real_front_rpm) {
    csv_file_ << sim_front_rpm - real_front_rpm;
  }
  csv_file_ << ",";
  if (has_real_motor_rpm_) {
    csv_file_ << sim_sensors.motor_rpm - latest_real_motor_rpm_;
  }
  csv_file_ << "\n";
}
