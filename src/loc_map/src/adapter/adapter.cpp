#include "adapter/adapter.hpp"

#include "loc_map/lm_subscriber.hpp"

Adapter::Adapter(std::string mode, LMSubscriber* subscriber) {
  this->node = subscriber;

  if (mode == "eufs") {
    this->eufs_init();
  } else if (mode == "fsds") {
    this->fsds_init();
  } else if (mode == "ads_dv") {
    this->ads_dv_init();
  }
}

void Adapter::eufs_init() {
  this->node->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
      std::bind(&Adapter::imu_subscription_callback, this, std::placeholders::_1));
  this->node->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
      "ros_can/wheel_speeds", 10,
      std::bind(&Adapter::eufs_wheel_speeds_subscription_callback, this, std::placeholders::_1));

  // mission control
  this->node->create_subscription<eufs_msgs::msg::CanState>(
      "/ros_can/state", 10,
      std::bind(&Adapter::eufs_mission_state_callback, this, std::placeholders::_1));
  this->eufs_mission_state_client_ =
      this->node->create_client<eufs_msgs::srv::SetCanState>("/ros_can/set_mission");
  this->eufs_ebs_client_ = this->node->create_client<eufs_msgs::srv::SetCanState>("/ros_can/ebs");
}

void Adapter::fsds_init() {
  this->node->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
      std::bind(&Adapter::imu_subscription_callback, this, std::placeholders::_1));
  this->node->create_subscription<fs_msgs::msg::WheelStates>(
      "/wheel_states", 10,
      std::bind(&Adapter::fsds_wheel_speeds_subscription_callback, this, std::placeholders::_1));

  // mission control
  this->node->create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", 10,
      std::bind(&Adapter::fsds_mission_state_callback, this, std::placeholders::_1));
  this->fsds_ebs_publisher_ =
      this->node->create_publisher<fs_msgs::msg::FinishedSignal>("/signal/finished", 10);
}

void Adapter::ads_dv_init() {}

void Adapter::imu_subscription_callback(const sensor_msgs::msg::Imu msg) {
  double angular_velocity = msg.angular_velocity.z;
  double acceleration_x = msg.linear_acceleration.x;
  double acceleration_y = msg.linear_acceleration.y;

  this->node->_imu_subscription_callback(angular_velocity, acceleration_x, acceleration_y);
}

void Adapter::eufs_mission_state_callback(const eufs_msgs::msg::CanState msg) {
  RCLCPP_INFO(this->node->get_logger(), "I heard: '%d' and '%d'", msg.ami_state, msg.as_state);

  auto mission = msg.ami_state;

  if (mission == eufs_msgs::msg::CanState::AMI_ACCELERATION) {
    this->node->set_mission(Mission::acceleration);
  } else if (mission == eufs_msgs::msg::CanState::AMI_SKIDPAD) {
    this->node->set_mission(Mission::skidpad);
  } else if (mission == eufs_msgs::msg::CanState::AMI_TRACK_DRIVE) {
    this->node->set_mission(Mission::trackdrive);
  } else if (mission == eufs_msgs::msg::CanState::AMI_AUTOCROSS) {
    this->node->set_mission(Mission::autocross);
  }
}

void Adapter::fsds_mission_state_callback(const fs_msgs::msg::GoSignal msg) {
  std::string mission = msg.mission;

  if (mission == "acceleration") {
    this->node->set_mission(Mission::acceleration);
  } else if (mission == "skidpad") {
    this->node->set_mission(Mission::skidpad);
  } else if (mission == "trackdrive") {
    this->node->set_mission(Mission::trackdrive);
  } else if (mission == "autocross") {
    this->node->set_mission(Mission::autocross);
  }
}

void Adapter::eufs_wheel_speeds_subscription_callback(
    const eufs_msgs::msg::WheelSpeedsStamped msg) {
  this->node->_wheel_speeds_subscription_callback(msg.speeds.lb_speed, msg.speeds.lf_speed,
                                                  msg.speeds.rb_speed, msg.speeds.rf_speed,
                                                  msg.speeds.steering);
}

void Adapter::fsds_wheel_speeds_subscription_callback(const fs_msgs::msg::WheelStates msg) {
  float steering_angle = (msg.fl_steering_angle + msg.fr_steering_angle) / 2.0;
  this->node->_wheel_speeds_subscription_callback(msg.rl_rpm, msg.fl_rpm, msg.rr_rpm, msg.fr_rpm,
                                                  steering_angle);
}

void Adapter::eufs_set_mission_state(int mission, int state) {
  auto request = std::make_shared<eufs_msgs::srv::SetCanState::Request>();
  request->ami_state = mission;
  request->as_state = state;

  auto result_future = this->eufs_mission_state_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->node->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->node->get_logger(), "Failed to call service");
  }
}