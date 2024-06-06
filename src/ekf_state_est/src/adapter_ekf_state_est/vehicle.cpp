#include "adapter_ekf_state_est/vehicle.hpp"

#include "common_lib/competition_logic/color.hpp"
#include "ros_node/se_node.hpp"

VehicleAdapter::VehicleAdapter(bool use_odometry, bool use_simulated_perception,
                               std::string motion_model_name,
                               std::string data_assocation_model_name, float sml_da_curvature,
                               float sml_initial_limit, float observation_noise,
                               float wheel_speed_sensor_noise,
                               float data_association_limit_distance)
    : SENode(use_odometry, use_simulated_perception, motion_model_name, data_assocation_model_name,
             sml_da_curvature, sml_initial_limit, observation_noise, wheel_speed_sensor_noise,
             data_association_limit_distance) {
  this->_operational_status_subscription_ =
      this->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          [this](const custom_interfaces::msg::OperationalStatus::SharedPtr msg) {
            RCLCPP_DEBUG(this->get_logger(), "Operational status received. Mission: %d - Go: %d",
                         msg->as_mission, msg->go_signal);
            this->_go_ = msg->go_signal;
            this->_mission_ = common_lib::competition_logic::Mission(msg->as_mission);
          });
  this->_yaw_accy_imu_subscription_.subscribe(this, "/vehicle/imu_yaw_acc_y");
  this->_roll_accx_imu_subscription_.subscribe(this, "/vehicle/imu_roll_acc_x");
  const ImuPolicy imu_policy(10);
  this->_imu_sync_ = std::make_shared<message_filters::Synchronizer<ImuPolicy>>(
      imu_policy, _yaw_accy_imu_subscription_, _roll_accx_imu_subscription_);
  this->_imu_sync_->registerCallback(&VehicleAdapter::vehicle_imu_subscription_callback, this);

  this->_rl_wheel_rpm_subscription_.subscribe(this, "/vehicle/rl_rpm");
  this->_rr_wheel_rpm_subscription_.subscribe(this, "/vehicle/rr_rpm");
  this->_steering_angle_subscription_.subscribe(this, "/vehicle/bosch_steering_angle");
  const WheelSteerPolicy policy(10);
  this->_sync_ = std::make_shared<message_filters::Synchronizer<WheelSteerPolicy>>(
      policy, _rl_wheel_rpm_subscription_, _rr_wheel_rpm_subscription_,
      _steering_angle_subscription_);
  this->_sync_->registerCallback(&VehicleAdapter::wheel_speeds_subscription_callback, this);

  this->_finished_client_ = this->create_client<std_srvs::srv::Trigger>("/as_srv/mission_finished");
}

void VehicleAdapter::wheel_speeds_subscription_callback(
    const custom_interfaces::msg::WheelRPM& rl_wheel_rpm_msg,
    const custom_interfaces::msg::WheelRPM& rr_wheel_rpm_msg,
    const custom_interfaces::msg::SteeringAngle& steering_angle_msg) {
  this->_wheel_speeds_subscription_callback(rl_wheel_rpm_msg.rl_rpm, rr_wheel_rpm_msg.rr_rpm, 0.0,
                                            0.0, steering_angle_msg.steering_angle,
                                            steering_angle_msg.header.stamp);
}

void VehicleAdapter::vehicle_imu_subscription_callback(
    const custom_interfaces::msg::ImuData& roll_accx_data,
    const custom_interfaces::msg::ImuData& yaw_accy_data) {
  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.angular_velocity.z = yaw_accy_data.gyro;
  imu_msg.linear_acceleration.x = roll_accx_data.acc;
  imu_msg.linear_acceleration.y = yaw_accy_data.acc;
  this->imu_subscription_callback(imu_msg);
}

// TODO: implement a more complex logic, like the one in inspection node
void VehicleAdapter::finish() {
  this->_finished_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (future.get()->success) {
          RCLCPP_INFO(this->get_logger(), "Finished signal sent");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to send finished signal");
        }
      });
}