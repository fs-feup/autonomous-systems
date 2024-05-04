#include "inspection_node/inspection_ros.hpp"

#include "exceptions/invalid_mission_exception.hpp"
#include "message_filters/message_traits.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "std_msgs/msg/string.hpp"

InspectionMission::InspectionMission() : Node("inspection") {
  inspection_object.turning_period = declare_parameter<double>("turning_period", 4.0);
  inspection_object.finish_time = declare_parameter<double>("finish_time", 26.0);
  inspection_object.wheel_radius = declare_parameter<double>("wheel_radius", 0.254);
  inspection_object.max_angle =
      declare_parameter<double>("max_angle", 0.52359877559);  // 30 degrees in rad
  inspection_object.start_and_stop = declare_parameter<bool>("start_and_stop", false);
  declare_parameter<double>("ebs_test_ideal_velocity", 2.0);
  declare_parameter<double>("ebs_test_gain", 0.25);
  declare_parameter<double>("inspection_ideal_velocity", 1.0);
  declare_parameter<double>("inspection_gain", 0.25);

  // creates publisher that should yield throttle/acceleration/...
  control_command_publisher =
      this->create_publisher<custom_interfaces::msg::ControlCommand>("/as_msgs/controls", 10);

  // creates client for the end of mission signal
  finish_client = this->create_client<std_srvs::srv::Trigger>("/as_srv/mission_finished");

  // creates client for the emergency signal
  emergency_client = this->create_client<std_srvs::srv::Trigger>("/as_srv/emergency");

  // get mission
  mission_signal_subscription =
      this->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", 10,
          std::bind(&InspectionMission::mission_decider, this, std::placeholders::_1));

  rl_rpm_subscription.subscribe(this, "/vehicle/rl_rpm");
  rr_rpm_subscription.subscribe(this, "/vehicle/rr_rpm");

  // WSS Synchronization
  const WSSPolicy policy(10);
  sync_ = std::make_shared<message_filters::Synchronizer<WSSPolicy>>(policy, rl_rpm_subscription,
                                                                     rr_rpm_subscription);
  sync_->registerCallback(&InspectionMission::inspection_script, this);

  RCLCPP_INFO(this->get_logger(), "Inspection node has been started.");
}

void InspectionMission::mission_decider(
    custom_interfaces::msg::OperationalStatus::SharedPtr mission_signal) {
  std::string mission_string = get_mission_string(mission_signal->as_mission);
  RCLCPP_DEBUG(this->get_logger(), "Mission received: %s", mission_string.c_str());

  try {
    if (mission_signal->as_mission == Mission::INSPECTION) {
      inspection_object.ideal_velocity = get_parameter("inspection_ideal_velocity").as_double();
      inspection_object.gain = get_parameter("inspection_gain").as_double();
      this->mission = Mission::INSPECTION;
    } else if (mission_signal->as_mission == Mission::EBS_TEST) {
      inspection_object.ideal_velocity = get_parameter("ebs_test_ideal_velocity").as_double();
      inspection_object.gain = get_parameter("ebs_test_gain").as_double();
      this->mission = Mission::EBS_TEST;
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid mission for inspection: %s", mission_string.c_str());
      return;
    }
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException& exception) {
    RCLCPP_ERROR(this->get_logger(), "Parameter not declared: %s", exception.what());
    return;
  }

  if (mission_signal->go_signal != go) {
    initial_time = this->clock.now();
  }
  go = mission_signal->go_signal;
}

void InspectionMission::inspection_script(const custom_interfaces::msg::WheelRPM& current_rl_rpm,
                                          const custom_interfaces::msg::WheelRPM& current_rr_rpm) {
  if (!go || mission == Mission::NONE) {
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Executing Inspection Script.");

  // initialization
  auto current_time = this->clock.now();
  auto elapsed_time = (current_time - initial_time).seconds();
  double average_rpm = (current_rl_rpm.rl_rpm + current_rr_rpm.rr_rpm) / 2.0;
  double current_velocity = inspection_object.rpm_to_velocity(average_rpm);

  // calculate steering
  double calculated_steering =
      mission == Mission::INSPECTION ? inspection_object.calculate_steering(elapsed_time) : 0;

  // if the time is over, the car should be stopped
  if (elapsed_time >= (inspection_object.finish_time)) {
    inspection_object.current_goal_velocity = 0;
  }

  // calculate throttle and convert to control command
  double calculated_throttle = inspection_object.calculate_throttle(current_velocity);
  RCLCPP_DEBUG(this->get_logger(), "Calculated throttle: %f", calculated_throttle);

  // publish suitable message
  if (elapsed_time < inspection_object.finish_time ||
      std::abs(current_velocity) > WHEELS_STOPPED_THRESHOLD) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Publishing control command. Steering: %f; Torque: %f; Elapsed Time: %f",
                 calculated_steering, calculated_throttle, elapsed_time);
    publish_controls(inspection_object.throttle_to_adequate_range(calculated_throttle),
                     calculated_steering);
  } else {
    // if the mission is over, publish the finish signal
    RCLCPP_DEBUG(this->get_logger(), "Mission is over. Stopping the car.");
    this->mission_end_timer = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&InspectionMission::end_of_mission, this));
  }

  // update ideal velocity if necessary
  inspection_object.redefine_goal_velocity(current_velocity);
}

void InspectionMission::publish_controls(double throttle, double steering) const {
  custom_interfaces::msg::ControlCommand control_command;
  control_command.throttle = throttle;
  control_command.steering = steering;
  control_command_publisher->publish(control_command);
}

// --------------------- END OF MISSION LOGIC -------------------------

void InspectionMission::end_of_mission() {
  RCLCPP_DEBUG(this->get_logger(), "Sending ending signal.");
  if (this->mission == Mission::INSPECTION) {
    this->finish_client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        std::bind(&InspectionMission::handle_end_of_mission_response, this, std::placeholders::_1));
  } else if (this->mission == Mission::EBS_TEST) {
    this->finish_client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        std::bind(&InspectionMission::handle_end_of_mission_response, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid mission at mission end: %s",
                 MISSION_STRING_MAP.find(this->mission)->second.c_str());
  }
}

void InspectionMission::handle_end_of_mission_response(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) const {
  auto result = future.get();
  if (result->success) {
    this->mission_end_timer->cancel();
    RCLCPP_INFO(this->get_logger(), "Mission has been finished.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Mission could not be finished.");
  }
}