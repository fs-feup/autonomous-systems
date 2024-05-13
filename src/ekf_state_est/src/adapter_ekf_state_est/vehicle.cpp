#include "adapter_ekf_state_est/vehicle.hpp"

#include "common_lib/competition_logic/color.hpp"
#include "ros_node/se_node.hpp"

VehicleAdapter::VehicleAdapter(std::shared_ptr<SENode> se_node) : Adapter(se_node) { this->init(); }

void VehicleAdapter::init() {
  this->_yaw_accy_imu_subscription_.subscribe(this->node, "/vehicle/imu_yaw_acc_y");
  this->_roll_accx_imu_subscription_.subscribe(this->node, "/vehicle/imu_roll_acc_x");
  const ImuPolicy imu_policy(10);
  this->_imu_sync_ = std::make_shared<message_filters::Synchronizer<ImuPolicy>>(
      imu_policy, _yaw_accy_imu_subscription_, _roll_accx_imu_subscription_);
  this->_imu_sync_->registerCallback(&VehicleAdapter::imu_subscription_callback, this);

  this->_rl_wheel_rpm_subscription_.subscribe(this->node, "/vehicle/rl_rpm");
  this->_rr_wheel_rpm_subscription_.subscribe(this->node, "/vehicle/rr_rpm");
  this->_steering_angle_subscription_.subscribe(this->node, "/vehicle/bosch_steering_angle");
  const WheelSteerPolicy policy(10);
  this->_sync_ = std::make_shared<message_filters::Synchronizer<WheelSteerPolicy>>(
      policy, _rl_wheel_rpm_subscription_, _rr_wheel_rpm_subscription_,
      _steering_angle_subscription_);
  this->_sync_->registerCallback(&VehicleAdapter::wheel_speeds_subscription_callback, this);

  this->_finished_client_ =
      this->node->create_client<std_srvs::srv::Trigger>("/as_srv/mission_finished");
}

void VehicleAdapter::wheel_speeds_subscription_callback(
    const custom_interfaces::msg::WheelRPM& rl_wheel_rpm_msg,
    const custom_interfaces::msg::WheelRPM& rr_wheel_rpm_msg,
    const custom_interfaces::msg::SteeringAngle& steering_angle_msg) {
  this->node->_wheel_speeds_subscription_callback(rl_wheel_rpm_msg.rl_rpm, rr_wheel_rpm_msg.rr_rpm,
                                                  0.0, 0.0, steering_angle_msg.steering_angle,
                                                  steering_angle_msg.header.stamp);
}

void VehicleAdapter::imu_subscription_callback(
    const custom_interfaces::msg::ImuData& roll_accx_data,
    const custom_interfaces::msg::ImuData& yaw_accy_data) {
  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.angular_velocity.z = yaw_accy_data.gyro;
  imu_msg.linear_acceleration.x = roll_accx_data.acc;
  imu_msg.linear_acceleration.y = yaw_accy_data.acc;
  this->node->_imu_subscription_callback(imu_msg);
}

// TODO: implement a more complex logic, like the one in inspection node
void VehicleAdapter::finish() {
  this->_finished_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (future.get()->success) {
          RCLCPP_INFO(this->node->get_logger(), "Finished signal sent");
        } else {
          RCLCPP_ERROR(this->node->get_logger(), "Failed to send finished signal");
        }
      });
}