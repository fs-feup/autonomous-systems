#include <ros_can.hpp>
#include <string>

using std::placeholders::_1;
using std::placeholders::_2;

CanInterface::CanInterface() : Node("ros_can") {
  // Declare ROS parameters
  can_debug_ = declare_parameter<int>("can_debug", can_debug_);
  simulate_can_ = declare_parameter<int>("simulate_can", simulate_can_);
  can_interface_ = declare_parameter<std::string>("can_interface", can_interface_);
  loop_rate = declare_parameter<int>("loop_rate", loop_rate);
  max_dec_ = declare_parameter<float>("max_dec", max_dec_);
  engine_threshold_ = declare_parameter<float>("engine_threshold", engine_threshold_);
  rpm_limit_ = declare_parameter<float>("rpm_limit", rpm_limit_);
  cmd_timeout_ = declare_parameter<double>("cmd_timeout", cmd_timeout_);
  if (declare_parameter<bool>("debug_logging", false)) {
    get_logger().set_level(rclcpp::Logger::Level::Debug);
  }

  // CAN interface setup
  if (can_debug_) RCLCPP_INFO(get_logger(), "Starting FS-AI CAN library in DEBUG MODE");
  if (simulate_can_) RCLCPP_INFO(get_logger(), "Simulating CAN data");
  fs_ai_api_init(const_cast<char *>(can_interface_.c_str()), can_debug_, simulate_can_);

  // ROS subscribers
  cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/cmd", 1, std::bind(&CanInterface::commandCallback, this, _1));
  flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/ros_can/mission_completed", 1, std::bind(&CanInterface::flagCallback, this, _1));
  driving_flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/state_machine/driving_flag", 1, std::bind(&CanInterface::drivingFlagCallback, this, _1));

  // ROS publishers
  state_pub_ = this->create_publisher<eufs_msgs::msg::CanState>("/ros_can/state", 1);
  state_pub_str_ = this->create_publisher<std_msgs::msg::String>("/ros_can/state_str", 1);
  wheel_pub_ =
      this->create_publisher<eufs_msgs::msg::WheelSpeedsStamped>("/ros_can/wheel_speeds", 1);
  vehicle_commands_pub_ = this->create_publisher<eufs_msgs::msg::VehicleCommandsStamped>(
      "/ros_can/vehicle_commands", 1);
  twist_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/ros_can/twist", 1);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/ros_can/imu", 1);
  fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/ros_can/fix", 1);

  // ROS services
  ebs_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/ros_can/ebs", std::bind(&CanInterface::requestEBS, this, _1, _2));

  // Setup ROS timer
  std::chrono::duration<float> rate(1 / static_cast<double>(loop_rate));
  timer_ = this->create_wall_timer(rate, std::bind(&CanInterface::loop, this));

  // Value of 0.0 means time is uninitialised
  last_cmd_message_time_ = 0.0;
}

void CanInterface::loop() {
  // Get fresh data from VCU
  fs_ai_api_vcu2ai_get_data(&vcu2ai_data_);
  fs_ai_api_gps_get_data(&gps_data_);
  fs_ai_api_imu_get_data(&imu_data_);

  // Log new data (in one string so log messages don't get separated)
  std::string msg_recv =
      "--- Read CAN data ---\n"
      "AS STATE:    " + std::to_string(vcu2ai_data_.VCU2AI_AS_STATE) + "\n" +
      "AMI STATE:   " + std::to_string(vcu2ai_data_.VCU2AI_AMI_STATE) + "\n" +
      "RR PULSE:    " + std::to_string(vcu2ai_data_.VCU2AI_RR_PULSE_COUNT) + "\n" +
      "RL PULSE:    " + std::to_string(vcu2ai_data_.VCU2AI_RL_PULSE_COUNT) + "\n" +
      "FR PULSE:    " + std::to_string(vcu2ai_data_.VCU2AI_FR_PULSE_COUNT) + "\n" +
      "FL PULSE:    " + std::to_string(vcu2ai_data_.VCU2AI_FL_PULSE_COUNT) + "\n" +
      "RR RPM:      " + std::to_string(vcu2ai_data_.VCU2AI_RR_WHEEL_SPEED_rpm) + "\n" +
      "RL RPM:      " + std::to_string(vcu2ai_data_.VCU2AI_RL_WHEEL_SPEED_rpm) + "\n" +
      "FR RPM:      " + std::to_string(vcu2ai_data_.VCU2AI_FR_WHEEL_SPEED_rpm) + "\n" +
      "FL RPM:      " + std::to_string(vcu2ai_data_.VCU2AI_FL_WHEEL_SPEED_rpm) + "\n" +
      "STEER ANGLE: " + std::to_string(vcu2ai_data_.VCU2AI_STEER_ANGLE_deg) + "\n";
  RCLCPP_DEBUG(get_logger(), "%s", msg_recv.c_str());

  // Update AS state
  as_state_ = vcu2ai_data_.VCU2AI_AS_STATE;

  // Reset state variables when in AS_OFF
  if (as_state_ == fs_ai_api_as_state_e::AS_OFF) {
    ebs_state_ = fs_ai_api_estop_request_e::ESTOP_NO;
    driving_flag_ = false;
    mission_complete_ = false;
    steering_ = 0;
    torque_ = 0;
    rpm_request_ = 0.0;
    braking_ = 0;
    last_cmd_message_time_ = 0;
  }

  // publish all received data
  wheel_pub_->publish(CanInterface::makeWsMessage(vcu2ai_data_));
  twist_pub_->publish(CanInterface::makeTwistMessage(vcu2ai_data_));
  fix_pub_->publish(CanInterface::makeGpsMessage(gps_data_));
  imu_pub_->publish(CanInterface::makeImuMessage(imu_data_));
  vehicle_commands_pub_->publish(CanInterface::makeVehicleCommandsMessage());

  // Read and publish state data
  auto state_msg = CanInterface::makeStateMessage(vcu2ai_data_);
  state_pub_->publish(state_msg);
  state_pub_str_->publish(makeStateString(state_msg));

  // Assign data to be sent
  ai2vcu_data_.AI2VCU_ESTOP_REQUEST = ebs_state_;
  ai2vcu_data_.AI2VCU_BRAKE_PRESS_REQUEST_pct = braking_;
  ai2vcu_data_.AI2VCU_AXLE_TORQUE_REQUEST_Nm = torque_;
  ai2vcu_data_.AI2VCU_STEER_ANGLE_REQUEST_deg = steering_;
  ai2vcu_data_.AI2VCU_AXLE_SPEED_REQUEST_rpm = rpm_request_;
  ai2vcu_data_.AI2VCU_HANDSHAKE_SEND_BIT = CanInterface::getHandshake(vcu2ai_data_);
  ai2vcu_data_.AI2VCU_DIRECTION_REQUEST = CanInterface::getDirectionReq(vcu2ai_data_);
  ai2vcu_data_.AI2VCU_MISSION_STATUS = CanInterface::getMissionStatus(vcu2ai_data_);

  // Log sent data (in one string so log messages don't get separated)
  std::string msg_send =
      "--- Sending CAN data ---\n"
      "EBS:            " + std::to_string(ai2vcu_data_.AI2VCU_ESTOP_REQUEST) + "\n" +
      "Brake pct:      " + std::to_string(ai2vcu_data_.AI2VCU_BRAKE_PRESS_REQUEST_pct) + "\n" +
      "Steering:       " + std::to_string(ai2vcu_data_.AI2VCU_STEER_ANGLE_REQUEST_deg) + "\n" +
      "Torque:         " + std::to_string(ai2vcu_data_.AI2VCU_AXLE_TORQUE_REQUEST_Nm) + "\n" +
      "Axle rpm:       " + std::to_string(ai2vcu_data_.AI2VCU_AXLE_SPEED_REQUEST_rpm) + "\n" +
      "Direction req:  " + std::to_string(ai2vcu_data_.AI2VCU_DIRECTION_REQUEST) + "\n" +
      "Mission status: " + std::to_string(ai2vcu_data_.AI2VCU_MISSION_STATUS) + "\n";
  RCLCPP_DEBUG(get_logger(), "%s", msg_send.c_str());

  // Send data to car
  fs_ai_api_ai2vcu_set_data(&ai2vcu_data_);

  // Only check timeout if driving_flag is true
  if (driving_flag_) {
    checkTimeout();
  }
}

fs_ai_api_handshake_send_bit_e CanInterface::getHandshake(const fs_ai_api_vcu2ai_struct data) {
  auto handshake = data.VCU2AI_HANDSHAKE_RECEIVE_BIT;
  if (handshake == fs_ai_api_handshake_receive_bit_e::HANDSHAKE_RECEIVE_BIT_OFF)
    return fs_ai_api_handshake_send_bit_e::HANDSHAKE_SEND_BIT_OFF;
  else
    return fs_ai_api_handshake_send_bit_e::HANDSHAKE_SEND_BIT_ON;
}

fs_ai_api_direction_request_e CanInterface::getDirectionReq(const fs_ai_api_vcu2ai_struct data) {
  if (data.VCU2AI_AS_STATE == fs_ai_api_as_state_e::AS_DRIVING && driving_flag_)
    return fs_ai_api_direction_request_e::DIRECTION_FORWARD;
  else
    return fs_ai_api_direction_request_e::DIRECTION_NEUTRAL;
}

fs_ai_api_mission_status_e CanInterface::getMissionStatus(const fs_ai_api_vcu2ai_struct data) {
  switch (data.VCU2AI_AS_STATE) {
    case fs_ai_api_as_state_e::AS_OFF:
      // Check whether a mission has been chosen, and acknowledge it if so
      if (data.VCU2AI_AMI_STATE != fs_ai_api_ami_state_e::AMI_NOT_SELECTED)
        return fs_ai_api_mission_status_e::MISSION_SELECTED;
      else
        return fs_ai_api_mission_status_e::MISSION_NOT_SELECTED;
    case fs_ai_api_as_state_e::AS_READY:
      if (driving_flag_) {
        // our stack is ready to start driving
        return fs_ai_api_mission_status_e::MISSION_RUNNING;
      } else {
        // still not ready to start driving
        return fs_ai_api_mission_status_e::MISSION_SELECTED;
      }
    case fs_ai_api_as_state_e::AS_DRIVING:
      if (mission_complete_) {
        // mission has been finished
        return fs_ai_api_mission_status_e::MISSION_FINISHED;
      } else {
        // still doing a mission
        return fs_ai_api_mission_status_e::MISSION_RUNNING;
      }
    case fs_ai_api_as_state_e::AS_FINISHED:
      return fs_ai_api_mission_status_e::MISSION_FINISHED;
    default:
      return fs_ai_api_mission_status_e::MISSION_NOT_SELECTED;
  }
}

void CanInterface::commandCallback(ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
  if (driving_flag_) {
    float acceleration = msg->drive.acceleration;
    // Divide by two as we have two motors (front and rear) which supply identical torque
    // 0.5 is an empirically measured constant for the rolling friction
    torque_ = (TOTAL_MASS_ * WHEEL_RADIUS_ * std::abs(acceleration + 0.5))/ 2.0;
    if (acceleration > 0.0) {
      braking_ = 0.0;
      rpm_request_ = rpm_limit_;  // Positive torque if the rpm_request is positive
    } else if (acceleration == 0.0) {
      torque_ = 0.0;
      braking_ = 0.0;
    } else if (acceleration > engine_threshold_) {  // Engine brake if possible
      braking_ = 0.0;
      rpm_request_ = 0.0;  // A negative torque is achieved by an rpm_request with a value of zero
    } else {  // If we want to decelerate faster than the engine allows, use the mechanical brakes
      torque_ = 0.0;  // Don't engine brake and mechanical brake simultaneously
      braking_ = (-acceleration / max_dec_) * MAX_BRAKE_;  // Braking needs to be positive
    }

    // Convert from radians to degrees
    steering_ = static_cast<float>(msg->drive.steering_angle * 180.0 / M_PI);
  } else {
    // Vehicle will not move to AS_DRIVING unless these are set to 0
    steering_ = 0;
    torque_ = 0;
    rpm_request_ = 0.0;

    // We don't need braking unless we are driving
    braking_ = 0;
  }

  steering_ = checkAndTrunc(steering_, MAX_STEERING_ANGLE_DEG_, "setting steering", false);
  torque_ = checkAndTrunc(torque_, MAX_TORQUE_, "setting torque");
  braking_ = checkAndTrunc(braking_, MAX_BRAKE_, "setting brake");

  // Update last message time
  last_cmd_message_time_ = this->now().seconds();
}

void CanInterface::flagCallback(std_msgs::msg::Bool::SharedPtr msg) {
  mission_complete_ = msg->data;
}

void CanInterface::drivingFlagCallback(std_msgs::msg::Bool::SharedPtr msg) {
  // Driving flag can only be set to true if we're in AS_DRIVING_
  if (msg->data && as_state_ == fs_ai_api_as_state_e::AS_DRIVING) {
    driving_flag_ = true;
  } else if (msg->data) {
    driving_flag_ = false;
    RCLCPP_WARN(get_logger(), "Driving flag is true but as_state is %i", as_state_);
  } else {
    driving_flag_ = msg->data;
  }

  if (driving_flag_ && last_cmd_message_time_ == 0.0) {
    // We start the cmd timeout on the positive edge of `driving_flag_`
    last_cmd_message_time_ = this->now().seconds();
  } else if (!driving_flag_) {
    // Reset last cmd message time back to magic value (0.0) if we stop driving
    last_cmd_message_time_ = 0.0;
  }
}

bool CanInterface::requestEBS(std_srvs::srv::Trigger::Request::SharedPtr,
                              std_srvs::srv::Trigger::Response::SharedPtr response) {
  RCLCPP_WARN(this->get_logger(), "Requesting EMERGENCY STOP");
  ebs_state_ = fs_ai_api_estop_request_e::ESTOP_YES;
  response->success = true;
  return response->success;
}

eufs_msgs::msg::VehicleCommandsStamped CanInterface::makeVehicleCommandsMessage() {
  auto msg = eufs_msgs::msg::VehicleCommandsStamped();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "base_footprint";

  // Populate msg
  msg.commands.ebs = ebs_state_;
  msg.commands.braking = braking_;
  msg.commands.torque = torque_;
  msg.commands.steering = steering_;
  msg.commands.rpm = rpm_request_;
  msg.commands.handshake = CanInterface::getHandshake(vcu2ai_data_);
  msg.commands.direction = CanInterface::getDirectionReq(vcu2ai_data_);
  msg.commands.mission_status = CanInterface::getMissionStatus(vcu2ai_data_);

  return msg;
}

eufs_msgs::msg::WheelSpeedsStamped CanInterface::makeWsMessage(const fs_ai_api_vcu2ai_struct data) {
  auto msg = eufs_msgs::msg::WheelSpeedsStamped();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "base_footprint";

  float steering_feedback = -data.VCU2AI_STEER_ANGLE_deg;  // inverted to match ISO convention
  float fl_speed = data.VCU2AI_FL_WHEEL_SPEED_rpm;
  float fr_speed = data.VCU2AI_FR_WHEEL_SPEED_rpm;
  float rl_speed = data.VCU2AI_RL_WHEEL_SPEED_rpm;
  float rr_speed = data.VCU2AI_RR_WHEEL_SPEED_rpm;

  // check values
  msg.speeds.lf_speed = checkAndTrunc(fl_speed, MAX_RPM_, "FL wheelspeed");
  msg.speeds.rf_speed = checkAndTrunc(fr_speed, MAX_RPM_, "FR wheelspeed");
  msg.speeds.lb_speed = checkAndTrunc(rl_speed, MAX_RPM_, "RL wheelspeed");
  msg.speeds.rb_speed = checkAndTrunc(rr_speed, MAX_RPM_, "RR wheelspeed");

  steering_feedback = checkAndTrunc(steering_feedback, MAX_STEERING_ANGLE_DEG_, "steering", false);
  msg.speeds.steering = (steering_feedback / 180) * M_PI;  // convert to radians

  return msg;
}

geometry_msgs::msg::TwistWithCovarianceStamped CanInterface::makeTwistMessage(
    const fs_ai_api_vcu2ai_struct data) {
  auto msg = geometry_msgs::msg::TwistWithCovarianceStamped();
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "base_footprint";

  auto wheel_speed = (checkAndTrunc(data.VCU2AI_RL_WHEEL_SPEED_rpm, MAX_RPM_, "RL ws") +
                      checkAndTrunc(data.VCU2AI_RR_WHEEL_SPEED_rpm, MAX_RPM_, "RR ws")) /
                     2;

  msg.twist.twist.linear.x = wheel_speed * M_PI * WHEEL_RADIUS_ / 30;

  auto steering_angle =
      checkAndTrunc(-data.VCU2AI_STEER_ANGLE_deg, MAX_STEERING_ANGLE_DEG_, "steering", false) /
      180 * M_PI;

  msg.twist.twist.angular.z = msg.twist.twist.linear.x * sin(steering_angle) / WHEELBASE_;

  msg.twist.covariance = {1e-9, 0, 0, 0, 0,    0,
                             0, 0, 0, 0, 0,    0,
                             0, 0, 0, 0, 0,    0,
                             0, 0, 0, 0, 0,    0,
                             0, 0, 0, 0, 0,    0,
                             0, 0, 0, 0, 0, 1e-9};

  return msg;
}

sensor_msgs::msg::Imu CanInterface::makeImuMessage(const fs_ai_api_imu_struct &data) {
  // Initialise message
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "base_footprint";

  // Get accelerations
  const float G_VALUE = 9.80665;
  msg.linear_acceleration.x = data.IMU_Acceleration_X_mG * 1000 * G_VALUE;
  msg.linear_acceleration.y = data.IMU_Acceleration_Y_mG * 1000 * G_VALUE;
  msg.linear_acceleration.z = data.IMU_Acceleration_Z_mG * 1000 * G_VALUE;

  // Get angular velocity
  msg.angular_velocity.x = (data.IMU_Rotation_X_degps / 180) * M_PI;
  msg.angular_velocity.y = (data.IMU_Rotation_Y_degps / 180) * M_PI;
  msg.angular_velocity.z = (data.IMU_Rotation_Z_degps / 180) * M_PI;

  return msg;
}

sensor_msgs::msg::NavSatFix CanInterface::makeGpsMessage(const fs_ai_api_gps_struct &data) {
  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "base_footprint";

  // Double check these with real values
  msg.altitude = data.GPS_Altitude;
  msg.latitude = data.GPS_Latitude_Degree + data.GPS_Latitude_Minutes / 60;
  msg.longitude = data.GPS_Longitude_Degree + data.GPS_Longitude_Minutes / 60;

  return msg;
}

eufs_msgs::msg::CanState CanInterface::makeStateMessage(const fs_ai_api_vcu2ai_struct &data) {
  eufs_msgs::msg::CanState msg;

  switch (data.VCU2AI_AS_STATE) {
    case fs_ai_api_as_state_e::AS_OFF:
      msg.as_state = eufs_msgs::msg::CanState::AS_OFF;
      break;
    case fs_ai_api_as_state_e::AS_READY:
      msg.as_state = eufs_msgs::msg::CanState::AS_READY;
      break;
    case fs_ai_api_as_state_e::AS_DRIVING:
      msg.as_state = eufs_msgs::msg::CanState::AS_DRIVING;
      break;
    case fs_ai_api_as_state_e::AS_EMERGENCY_BRAKE:
      msg.as_state = eufs_msgs::msg::CanState::AS_EMERGENCY_BRAKE;
      break;
    case fs_ai_api_as_state_e::AS_FINISHED:
      msg.as_state = eufs_msgs::msg::CanState::AS_FINISHED;
      break;
    default:
      msg.as_state = eufs_msgs::msg::CanState::AS_OFF;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Invalid AS state from vehicle.");
  }

  switch (data.VCU2AI_AMI_STATE) {
    case fs_ai_api_ami_state_e::AMI_NOT_SELECTED:
      msg.ami_state = eufs_msgs::msg::CanState::AMI_NOT_SELECTED;
      break;
    case fs_ai_api_ami_state_e::AMI_ACCELERATION:
      msg.ami_state = eufs_msgs::msg::CanState::AMI_ACCELERATION;
      break;
    case fs_ai_api_ami_state_e::AMI_SKIDPAD:
      msg.ami_state = eufs_msgs::msg::CanState::AMI_SKIDPAD;
      break;
    case fs_ai_api_ami_state_e::AMI_AUTOCROSS:
      msg.ami_state = eufs_msgs::msg::CanState::AMI_AUTOCROSS;
      break;
    case fs_ai_api_ami_state_e::AMI_TRACK_DRIVE:
      msg.ami_state = eufs_msgs::msg::CanState::AMI_TRACK_DRIVE;
      break;
    case fs_ai_api_ami_state_e::AMI_STATIC_INSPECTION_A:
      msg.ami_state = eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_A;
      break;
    case fs_ai_api_ami_state_e::AMI_STATIC_INSPECTION_B:
      msg.ami_state = eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_B;
      break;
    case fs_ai_api_ami_state_e::AMI_AUTONOMOUS_DEMO:
      msg.ami_state = eufs_msgs::msg::CanState::AMI_AUTONOMOUS_DEMO;
      break;
    default:
      msg.ami_state = eufs_msgs::msg::CanState::AMI_NOT_SELECTED;
      RCLCPP_WARN(this->get_logger(), "Invalid AMI state from vehicle.");
  }

  return msg;
}

std_msgs::msg::String CanInterface::makeStateString(eufs_msgs::msg::CanState &state) {
  std::string str1;
  std::string str2;
  std::string str3;

  switch (state.as_state) {
    case eufs_msgs::msg::CanState::AS_OFF:
      str1 = "AS:OFF";
      break;
    case eufs_msgs::msg::CanState::AS_READY:
      str1 = "AS:READY";
      break;
    case eufs_msgs::msg::CanState::AS_DRIVING:
      str1 = "AS:DRIVING";
      break;
    case eufs_msgs::msg::CanState::AS_FINISHED:
      str1 = "AS:FINISHED";
      break;
    case eufs_msgs::msg::CanState::AS_EMERGENCY_BRAKE:
      str1 = "AS:EMERGENCY";
      break;
    default:
      str1 = "NO_SUCH_MESSAGE";
  }

  switch (state.ami_state) {
    case eufs_msgs::msg::CanState::AMI_NOT_SELECTED:
      str2 = "AMI:NOT_SELECTED";
      break;
    case eufs_msgs::msg::CanState::AMI_ACCELERATION:
      str2 = "AMI:ACCELERATION";
      break;
    case eufs_msgs::msg::CanState::AMI_SKIDPAD:
      str2 = "AMI:SKIDPAD";
      break;
    case eufs_msgs::msg::CanState::AMI_AUTOCROSS:
      str2 = "AMI:AUTOCROSS";
      break;
    case eufs_msgs::msg::CanState::AMI_TRACK_DRIVE:
      str2 = "AMI:TRACKDRIVE";
      break;
    case eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_A:
      str2 = "AMI:INSPECTION";
      break;
    case eufs_msgs::msg::CanState::AMI_MANUAL:
      str2 = "AMI:MANUAL";
      break;
    default:
      str2 = "NO_SUCH_MESSAGE";
  }

  if (driving_flag_)
    str3 = "DRIVING:TRUE";
  else
    str3 = "DRIVING:FALSE";

  std_msgs::msg::String msg = std_msgs::msg::String();
  msg.data = str1 + " " + str2 + " " + str3;
  return msg;
}

// Checks if value exceeds max allowed value, if it does truncate it and warn
float CanInterface::checkAndTrunc(const float val, const float max_val, const std::string type,
                                  bool trunc_at_zero) {
  float min_val = trunc_at_zero ? 0 : -max_val;
  if (val > max_val) {
    RCLCPP_DEBUG(get_logger(), "%s was %f but max is %f", type.c_str(), val, max_val);
    return max_val;
  } else {
    if (val < min_val) {
      RCLCPP_DEBUG(get_logger(), "%s was %f but min is %f", type.c_str(), val, min_val);
      return min_val;
    }
  }
  return val;
}

int CanInterface::checkAndTrunc(const int val, const int max_val, std::string type,
                                bool trunc_at_zero) {
  // Replicated because casting to float from int could lose information
  int min_val = trunc_at_zero ? 0 : -max_val;
  if (val > max_val) {
    RCLCPP_DEBUG(get_logger(), "%s was %i but max is %i", type.c_str(), val, max_val);
    return max_val;
  } else {
    if (val < min_val) {
      RCLCPP_DEBUG(get_logger(), "%s was %i but min is %i", type.c_str(), val, min_val);
      return min_val;
    }
  }
  return val;
}

void CanInterface::checkTimeout() {
  // Engage EBS if the duration between last message time and now exceeds threshold
  if (this->now().seconds() - last_cmd_message_time_ > cmd_timeout_) {
    RCLCPP_ERROR(get_logger(), "/cmd sent nothing for %f seconds, requesting EMERGENCY STOP",
                 cmd_timeout_);
    ebs_state_ = fs_ai_api_estop_request_e::ESTOP_YES;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanInterface>());
  rclcpp::shutdown();
  return 0;
}
