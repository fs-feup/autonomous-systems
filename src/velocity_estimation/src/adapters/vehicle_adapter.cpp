#include "adapters/vehicle_adapter.hpp"

#include <cmath>

static constexpr double kDegToRad = M_PI / 180.0;
static constexpr bool kEulerInDegrees = true;

static std::array<double, 3> gravity_from_euler(double roll, double pitch, double gravity) {
  const double sin_roll = std::sin(roll);
  const double cos_roll = std::cos(roll);
  const double sin_pitch = std::sin(pitch);
  const double cos_pitch = std::cos(pitch);

  return {-gravity * sin_pitch, gravity * sin_roll * cos_pitch,
          gravity * cos_roll * cos_pitch};
}

static std::array<double, 3> map_imu_axes(const std::array<double, 3>& vec) {
  return {-vec[1], vec[0], vec[2]};
}

VehicleAdapter::VehicleAdapter(const VEParameters& parameters) : VENode(parameters) {
  this->_acceleration_subscription_.subscribe(this, "/imu/acceleration");
  this->_euler_subscription_.subscribe(this, "/filter/euler");
  this->_angular_velocity_subscription_.subscribe(this, "/imu/angular_velocity");
  const XsensImuPolicy xsens_imu_policy(10);
  this->_xsens_imu_sync_ = std::make_shared<message_filters::Synchronizer<XsensImuPolicy>>(
      xsens_imu_policy, _acceleration_subscription_, _angular_velocity_subscription_,
      _euler_subscription_);
  this->_xsens_imu_sync_->registerCallback(&VehicleAdapter::imu_callback, this);

  this->_fl_wheel_rpm_subscription_.subscribe(this, "/vehicle/fl_rpm");
  this->_fr_wheel_rpm_subscription_.subscribe(this, "/vehicle/fr_rpm");

  const WheelSSPolicy policy(10);
  this->_wss_sync_ = std::make_shared<message_filters::Synchronizer<WheelSSPolicy>>(
      policy, _fl_wheel_rpm_subscription_, _fr_wheel_rpm_subscription_);
  this->_wss_sync_->registerCallback(&VehicleAdapter::wss_callback, this);

  this->_steering_angle_sub_ = this->create_subscription<custom_interfaces::msg::SteeringAngle>(
      "/vehicle/bosch_steering_angle", 1,
      std::bind(&VehicleAdapter::steering_angle_callback, this, std::placeholders::_1));
  this->_resolver_sub_ = this->create_subscription<custom_interfaces::msg::WheelRPM>(
      "/vehicle/motor_rpm", 1,
      std::bind(&VehicleAdapter::resolver_callback, this, std::placeholders::_1));
}

void VehicleAdapter::wss_callback(const custom_interfaces::msg::WheelRPM& fl_wheel_rpm_msg,
                                  const custom_interfaces::msg::WheelRPM& fr_wheel_rpm_msg) {
  common_lib::sensor_data::WheelEncoderData wss_data;
  wss_data.rl_rpm = 0;
  wss_data.rr_rpm = 0;
  wss_data.fl_rpm = fl_wheel_rpm_msg.fl_rpm;
  wss_data.fr_rpm = fr_wheel_rpm_msg.fr_rpm;
  if (fl_wheel_rpm_msg.fl_rpm > 2000 || fr_wheel_rpm_msg.fr_rpm > 2000) {
    RCLCPP_WARN(this->get_logger(), "Wheel RPM is too high");
    return;
  }
  this->_velocity_estimator_->wss_callback(wss_data);
  this->publish_velocities();
}

void VehicleAdapter::steering_angle_callback(const custom_interfaces::msg::SteeringAngle msg) {
  this->_velocity_estimator_->steering_callback(msg.steering_angle);
  this->publish_velocities();
}

void VehicleAdapter::imu_callback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr& acceleration_msg,
    const geometry_msgs::msg::Vector3Stamped::SharedPtr& angular_velocity_msg,
    const geometry_msgs::msg::Vector3Stamped::SharedPtr& euler_msg) {
  common_lib::sensor_data::ImuData imu_data;

  double roll = euler_msg->vector.x;
  double pitch = euler_msg->vector.y;
  double yaw = euler_msg->vector.z;
  if (kEulerInDegrees) {
    roll *= kDegToRad;
    pitch *= kDegToRad;
    yaw *= kDegToRad;
  }
  (void)yaw;

  const std::array<double, 3> acceleration = {
      acceleration_msg->vector.x, acceleration_msg->vector.y, acceleration_msg->vector.z};

  if (!this->imu_calibrated_) {
    const double accel_norm =
        std::sqrt(acceleration[0] * acceleration[0] + acceleration[1] * acceleration[1] +
                  acceleration[2] * acceleration[2]);
    const std::array<double, 3> g_unit = gravity_from_euler(roll, pitch, 1.0);
    this->gravity_norm_sum_ += accel_norm;
    this->accel_sum_[0] += acceleration[0];
    this->accel_sum_[1] += acceleration[1];
    this->accel_sum_[2] += acceleration[2];
    this->gravity_unit_sum_[0] += g_unit[0];
    this->gravity_unit_sum_[1] += g_unit[1];
    this->gravity_unit_sum_[2] += g_unit[2];

    this->average_imu_bias =
        (this->average_imu_bias * this->number_of_imu_readings + angular_velocity_msg->vector.z) /
        (this->number_of_imu_readings + 1);
    this->number_of_imu_readings++;

    this->imu_calibration_count_++;
    if (this->imu_calibration_count_ < this->imu_calibration_samples_) {
      return;
    }

    this->gravity_magnitude_ = this->gravity_norm_sum_ / this->imu_calibration_count_;
    const std::array<double, 3> accel_avg = {this->accel_sum_[0] / this->imu_calibration_count_,
                                             this->accel_sum_[1] / this->imu_calibration_count_,
                                             this->accel_sum_[2] / this->imu_calibration_count_};
    const std::array<double, 3> g_unit_avg = {
        this->gravity_unit_sum_[0] / this->imu_calibration_count_,
        this->gravity_unit_sum_[1] / this->imu_calibration_count_,
        this->gravity_unit_sum_[2] / this->imu_calibration_count_};
    this->accel_bias_ = {accel_avg[0] - this->gravity_magnitude_ * g_unit_avg[0],
                         accel_avg[1] - this->gravity_magnitude_ * g_unit_avg[1],
                         accel_avg[2] - this->gravity_magnitude_ * g_unit_avg[2]};
    this->imu_calibrated_ = true;
    RCLCPP_INFO(this->get_logger(), "IMU calibration complete. Estimated gravity: %.5f m/s^2",
                this->gravity_magnitude_);
    return;
  }

  const std::array<double, 3> gravity_imu = gravity_from_euler(roll, pitch, gravity_magnitude_);
  std::array<double, 3> linear_acc = {
      acceleration[0] - gravity_imu[0] - accel_bias_[0],
      acceleration[1] - gravity_imu[1] - accel_bias_[1],
      acceleration[2] - gravity_imu[2] - accel_bias_[2]};
  linear_acc = map_imu_axes(linear_acc);

  imu_data.rotational_velocity = angular_velocity_msg->vector.z - this->average_imu_bias;
  imu_data.acceleration_x = linear_acc[0];
  imu_data.acceleration_y = 0.0;
  imu_data.timestamp_ = acceleration_msg->header.stamp;

  this->_velocity_estimator_->imu_callback(imu_data);
  this->publish_velocities();
}

void VehicleAdapter::resolver_callback(custom_interfaces::msg::WheelRPM msg) {
  this->_velocity_estimator_->motor_rpm_callback(msg.rr_rpm);
  this->publish_velocities();
}
