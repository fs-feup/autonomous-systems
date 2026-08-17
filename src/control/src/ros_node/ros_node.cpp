#include "ros_node/ros_node.hpp"

using namespace common_lib::structures;

ControlNode::ControlNode(const ControlParameters& params)
    : Node("control"),
      params_(params),
      _execution_times_(std::make_shared<std::vector<double>>(5, 0.0)),
      controller_(controller_map.at(params_.controller_)(params)),
      execution_time_pub_(create_publisher<std_msgs::msg::Float64MultiArray>(
          "/control/execution_time", 10)),
      control_timer_(create_wall_timer(std::chrono::milliseconds(this->params_.command_time_interval_),
                              std::bind(&ControlNode::control_timer_callback, this))),
      path_point_array_sub_(create_subscription<custom_interfaces::msg::PathPointArray>(
          params.use_simulated_planning_ ? "/path_planning/mock_path" : "/path_planning/path",
          rclcpp::QoS(10),
          std::bind(&ControlNode::path_callback, this, std::placeholders::_1))) {
  RCLCPP_INFO(this->get_logger(), "Control Node Initialized with %s controller",
              params_.controller_.c_str());
  if (!params_.using_simulated_slam_) {
    vehicle_pose_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
        "/state_estimation/vehicle_pose", 10,
        std::bind(&ControlNode::vehicle_pose_callback, this, std::placeholders::_1));
  }
  if (!params_.using_simulated_velocities_) {
    velocity_sub_ = this->create_subscription<custom_interfaces::msg::Velocities>(
        "/state_estimation/velocities", 10,
        [this](const custom_interfaces::msg::Velocities& msg) {
          this->state.velocity_x = msg.velocity_x;
          this->state.velocity_y = msg.velocity_y;
          this->state.yaw_rate = msg.angular_velocity;
          this->vehicle_state_callback(this->state);
        });
  }
  steering_angle_sub_ = this->create_subscription<custom_interfaces::msg::SteeringAngle>(
      "/vehicle/steering_motor_state", 10, [this](const custom_interfaces::msg::SteeringAngle& msg) {
        this->state.steering_angle = msg.steering_angle;
        this->vehicle_state_callback(this->state);
      });
  remote_ebs_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/remote/ebs", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::Bool& msg) {
        if (remote_ebs_ != msg.data) {
          RCLCPP_INFO(this->get_logger(), "Remote EBS %s", msg.data ? "active" : "inactive");
        }
        remote_ebs_ = msg.data;
      });
  remote_take_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/remote/take_control", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::Bool& msg) {
        if (remote_take_control_ != msg.data) {
          RCLCPP_INFO(this->get_logger(), "Remote manual control %s",
                      msg.data ? "active" : "inactive");
        }
        remote_take_control_ = msg.data;
      });
}

void ControlNode::control_timer_callback() {
  if (!go_signal_) return;
  if (remote_ebs_ || remote_take_control_) return;

  rclcpp::Time start = this->now();
  common_lib::structures::ControlCommand command = this->controller_->get_control_command();
  double execution_time = (this->now() - start).seconds() * 1000;

  this->publish_command(command);

  this->_execution_times_->at(0) = execution_time;
  this->_execution_times_->at(1) = (this->_execution_times_->at(1) * static_cast<double>(this->number_of_loops_executed_) + execution_time) / (static_cast<double>(this->number_of_loops_executed_ + 1));
  this->number_of_loops_executed_++;
  std_msgs::msg::Float64MultiArray execution_time_msg;
  execution_time_msg.data = *this->_execution_times_;
  this->execution_time_pub_->publish(execution_time_msg);
  this->controller_->publish_solver_data(shared_from_this(), publisher_map_);
}

// This function is called when a new pose is received
void ControlNode::vehicle_pose_callback(const custom_interfaces::msg::Pose& vehicle_pose_msg) {
  this->controller_->vehicle_pose_callback(vehicle_pose_msg);
}

void ControlNode::path_callback(const custom_interfaces::msg::PathPointArray& path_msg) {
  this->controller_->path_callback(path_msg);
}


void ControlNode::vehicle_state_callback(const custom_interfaces::msg::VehicleStateVector& vel_msg) {
  this->controller_->vehicle_state_callback(vel_msg);
}

void ControlNode::create_vehicle_state_adapter() {
  
}
