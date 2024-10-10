#include "inspection_node/inspection_ros.hpp"

#include "exceptions/invalid_mission_exception.hpp"
#include "message_filters/message_traits.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"

InspectionMission::InspectionMission() : Node("inspection") {
  _inspection_object_.turning_period_ = declare_parameter<double>("turning_period", 4.0);
  _inspection_object_.finish_time_ = declare_parameter<double>("finish_time", 26.0);
  _inspection_object_.wheel_radius_ = declare_parameter<double>("wheel_radius", 0.254);
  _inspection_object_.max_angle_ =
      declare_parameter<double>("max_angle", MAX_ANGLE);  // 22.5 degrees in rad
  _inspection_object_.start_and_stop_ = declare_parameter<bool>("start_and_stop", false);
  declare_parameter<double>("ebs_test_ideal_velocity", 2.0);
  declare_parameter<double>("ebs_test_gain", 0.25);
  declare_parameter<double>("inspection_ideal_velocity", 1.0);
  declare_parameter<double>("inspection_gain", 0.25);

  // creates publisher that should yield throttle/acceleration/...
  _control_command_publisher_ =
      this->create_publisher<custom_interfaces::msg::ControlCommand>("/as_msgs/controls", 10);

  // creates client for the end of mission signal
  _finish_client_ = this->create_client<std_srvs::srv::Trigger>("/as_srv/mission_finished");

  // creates client for the emergency signal
  _emergency_client_ = this->create_client<std_srvs::srv::Trigger>("/as_srv/emergency");

  // get mission
  _mission_signal_subscription_ =
      this->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          std::bind(&InspectionMission::mission_decider, this, std::placeholders::_1));

  _timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&InspectionMission::inspection_script, this));  // Timer for end of mission

  _motor_rpm_subscription_ = this->create_subscription<custom_interfaces::msg::WheelRPM>(
        "/vehicle/motor_rpm", 10,
        std::bind(&InspectionMission::update_rpms_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Inspection node has been started.");
}

bool finished = false;

void InspectionMission::mission_decider(
    custom_interfaces::msg::OperationalStatus::SharedPtr mission_signal) {
  std::string mission_string =
      common_lib::competition_logic::get_mission_string(mission_signal->as_mission);
  // RCLCPP_DEBUG(this->get_logger(), "Mission received: %s", mission_string.c_str());

  try {
    if (mission_signal->as_mission == common_lib::competition_logic::Mission::INSPECTION) {
      _inspection_object_.ideal_velocity_ = get_parameter("inspection_ideal_velocity").as_double();
      _inspection_object_.gain_ = get_parameter("inspection_gain").as_double();
      this->_mission_ = common_lib::competition_logic::Mission::INSPECTION;
    } else if (mission_signal->as_mission == common_lib::competition_logic::Mission::EBS_TEST) {
      _inspection_object_.ideal_velocity_ = get_parameter("ebs_test_ideal_velocity").as_double();
      _inspection_object_.gain_ = get_parameter("ebs_test_gain").as_double();
      this->_mission_ = common_lib::competition_logic::Mission::EBS_TEST;
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid mission for inspection: %s", mission_string.c_str());
      return;
    }
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException& exception) {
    RCLCPP_ERROR(this->get_logger(), "Parameter not declared: %s", exception.what());
    return;
  }

  if (mission_signal->go_signal != _go_ && mission_signal->go_signal) {
    RCLCPP_INFO(this->get_logger(), "Starting timer");
    _initial_time_ = this->_clock_.now();
    _inspection_object_.stop_oscilating_ = false;
    _inspection_object_.current_goal_velocity_ = _inspection_object_.ideal_velocity_; 
    this->_mission_end_timer_started_ = false;
    this->_car_stopped_ = false;
  }
  _go_ = mission_signal->go_signal;
}

void InspectionMission::update_rpms_callback(const custom_interfaces::msg::WheelRPM& motor_rpm) {
  RCLCPP_DEBUG(this->get_logger(), "Motor RPM: %f",motor_rpm.rr_rpm);
  _motor_rpm_ = motor_rpm.rl_rpm / 4.0;
}

void InspectionMission::inspection_script() {
  if (!_go_ || _mission_ == common_lib::competition_logic::Mission::NONE) {
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Executing Inspection Script.");

  if (this->_car_stopped_) {
    // if the mission is over, publish the finish signal
    RCLCPP_DEBUG(this->get_logger(), "Mission is over. Stopping the car.");
    if (!this->_mission_end_timer_started_) {
      this->_mission_end_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(500), std::bind(&InspectionMission::end_of_mission, this));
      this->_mission_end_timer_started_ = true;
    }
    return;
  }

  // initialization
  auto current_time = this->_clock_.now();
  auto elapsed_time = (current_time - _initial_time_).seconds();
  double average_rpm = _motor_rpm_ / 4;
  RCLCPP_DEBUG(this->get_logger(), "average_rpm %f", average_rpm);
  double current_velocity = _inspection_object_.rpm_to_velocity(average_rpm);
  RCLCPP_DEBUG(this->get_logger(), "current_velocity %f", current_velocity);
  
  // calculate steering
  double calculated_steering = _mission_ == common_lib::competition_logic::Mission::INSPECTION
                                   ? _inspection_object_.calculate_steering(elapsed_time)
                                   : 0;

  // if the time is over, the car should be stopped
  bool steering_straight = calculated_steering < 0.1 && calculated_steering > -0.1;
  if (elapsed_time >= (_inspection_object_.finish_time_)) {
    _inspection_object_.current_goal_velocity_ = 0;
    if (steering_straight) {
      _inspection_object_.stop_oscilating_ = true;
    } 
  }

  // calculate throttle and convert to control command
  double calculated_throttle = _inspection_object_.calculate_throttle(current_velocity);

  if (elapsed_time >= _inspection_object_.finish_time_ && 
    std::abs(current_velocity) <= WHEELS_STOPPED_THRESHOLD) {
        calculated_throttle = 0.0;
      }
  
  //if (elapsed_time >= _inspection_object_.finish_time_){
   //// calculated_throttle = 0.0;
  //}
  
  if (elapsed_time >= _inspection_object_.finish_time_ && steering_straight) {
    calculated_steering = 0.0;
  }

  if (elapsed_time >= _inspection_object_.finish_time_ &&
      std::abs(current_velocity) <= WHEELS_STOPPED_THRESHOLD && steering_straight) {
      this->_car_stopped_ = true;
  }

  // publish suitable message
  double scaled_throttle = _inspection_object_.throttle_to_adequate_range(calculated_throttle); 
  publish_controls(scaled_throttle,calculated_steering);
  RCLCPP_DEBUG(this->get_logger(),
                 "Publishing control command. Steering: %lf; Torque after conversion - before: %lf - %lf; Elapsed Time: %lf",
                 calculated_steering, scaled_throttle, calculated_throttle, elapsed_time);
                 
  // update ideal velocity if necessary
  _inspection_object_.redefine_goal_velocity(current_velocity);
}

void InspectionMission::publish_controls(double throttle, double steering) const {
  custom_interfaces::msg::ControlCommand control_command;
  control_command.throttle = throttle;
  control_command.steering = steering;
  _control_command_publisher_->publish(control_command);
}

// --------------------- END OF MISSION LOGIC -------------------------

void InspectionMission::end_of_mission() {
  RCLCPP_INFO(this->get_logger(), "Sending ending signal...");
  if (this->_mission_ == common_lib::competition_logic::Mission::INSPECTION) {
    this->_finish_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        std::bind(&InspectionMission::handle_end_of_mission_response, this, std::placeholders::_1));
  } else if (this->_mission_ == common_lib::competition_logic::Mission::EBS_TEST) {
    this->_emergency_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        std::bind(&InspectionMission::handle_end_of_mission_response, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(
        this->get_logger(), "Invalid mission at mission end: %s",
        common_lib::competition_logic::MISSION_STRING_MAP.find(this->_mission_)->second.c_str());
  }
}

void InspectionMission::handle_end_of_mission_response(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) const {
  auto result = future.get();
  if (result->success) {
    this->_mission_end_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Mission has been finished!");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Mission could not be finished!");
  }
}
