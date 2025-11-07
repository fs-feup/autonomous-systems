#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_interfaces/msg/path.hpp>
#include <custom_interfaces/msg/robot_status.hpp>
#include <custom_interfaces/msg/robot_control.hpp>
#include <custom_interfaces/msg/state_estimation_debug.hpp>
#include <vector>
#include <string>
#include <cmath>


struct Point {
  int x, y;
  Point(int x_ = 0, int y_ = 0) : x(x_), y(y_) {}
  bool operator==(const Point& other) const {
    return x == other.x && y == other.y;
  }
};

class ControlNode : public rclcpp::Node {
public:
  ControlNode() : Node("control_node"),
                  current_waypoint_idx_(0),
                  path_received_(false),
                  position_received_(false),
                  status_received_(false),
                  waiting_for_movement_(false),
                  command_sent_time_(this->now()) {
    
    // parametro para controlos por segundo
    this->declare_parameter<double>("control_rate", 2.0);
    
    double control_rate = this->get_parameter("control_rate").as_double();
    int timer_ms = static_cast<int>(1000.0 / control_rate);
    
    path_sub_ = this->create_subscription<custom_interfaces::msg::Path>(
        "planning/path", 10,
        std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    
    position_sub_ = this->create_subscription<custom_interfaces::msg::StateEstimationDebug>(
        "state_estimation/debug", 10,  
        std::bind(&ControlNode::positionCallback, this, std::placeholders::_1));
    
    status_sub_ = this->create_subscription<custom_interfaces::msg::RobotStatus>(
        "robot/status", 10,
        std::bind(&ControlNode::statusCallback, this, std::placeholders::_1));
    
    cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
        "control/cmd", 10);
    
    control_pub_ = this->create_publisher<custom_interfaces::msg::RobotControl>(
        "control/status", 10);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&ControlNode::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Control node initialized with rate: %.2f Hz", control_rate);
  }

private:
  void pathCallback(const custom_interfaces::msg::Path::SharedPtr msg) {
    if (msg->points.empty()) {
      return;
    }
    
    path_.clear();
    for (const auto& pt : msg->points) {
      path_.push_back(Point(pt.x, pt.y));
    }
    
    current_waypoint_idx_ = 0;
    path_received_ = true;
    waiting_for_movement_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", path_.size());
  }
  
  void positionCallback(const custom_interfaces::msg::StateEstimationDebug::SharedPtr msg) {
    robot_pos_.x = msg->robot_grid_x;
    robot_pos_.y = msg->robot_grid_y;
    position_received_ = true;
}
  
  void statusCallback(const custom_interfaces::msg::RobotStatus::SharedPtr msg) {
    holding_box_ = msg->holding_box;
    status_received_ = true;
  }
  
  void controlLoop() {
    if (!path_received_ || !position_received_ || !status_received_) {
      return;
    }
    
    if (current_waypoint_idx_ >= path_.size()) {
      return;
    }
    
    // Waiting pela confirmacao que robot moveu
    if (waiting_for_movement_) {
      if (robot_pos_ == expected_pos_) {
        waiting_for_movement_ = false;
        RCLCPP_INFO(this->get_logger(), "Robot reached position (%d, %d)", 
                    robot_pos_.x, robot_pos_.y);
      } else {
        auto now = this->now();
        auto elapsed = (now - command_sent_time_).seconds();
        if (elapsed > 2.0) {  // timeout 2 segundos
          RCLCPP_WARN(this->get_logger(), 
                      "Movement timeout! Expected (%d, %d) but at (%d, %d)", 
                      expected_pos_.x, expected_pos_.y, robot_pos_.x, robot_pos_.y);
          waiting_for_movement_ = false;
        }
      }
      return;
    }
    
    Point target = path_[current_waypoint_idx_];

    if (robot_pos_ == target) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu at (%d, %d)", 
                  current_waypoint_idx_, target.x, target.y);
      current_waypoint_idx_++;
      return;
    }
    
    if (shouldPick(target)) {
      sendCommand("PICK", target);
      current_waypoint_idx_++;  
      return;
    }
    
    if (shouldDrop(target)) {
      sendCommand("DROP", target);
      current_waypoint_idx_++;  
      return;
    }
    
    std::string direction = calculateDirection(robot_pos_, target);
    
    if (direction.empty()) {
      RCLCPP_ERROR(this->get_logger(), 
                   "Cannot determine direction from (%d, %d) to (%d, %d)", 
                   robot_pos_.x, robot_pos_.y, target.x, target.y);
      current_waypoint_idx_++;
      return;
    }
    
    sendCommand(direction, target);
    
    if (direction == "UP") {
      expected_pos_ = Point(robot_pos_.x, robot_pos_.y + 1);
    } else if (direction == "DOWN") {
      expected_pos_ = Point(robot_pos_.x, robot_pos_.y - 1);
    } else if (direction == "LEFT") {
      expected_pos_ = Point(robot_pos_.x - 1, robot_pos_.y);
    } else if (direction == "RIGHT") {
      expected_pos_ = Point(robot_pos_.x + 1, robot_pos_.y);
    }
    
    waiting_for_movement_ = true;
    command_sent_time_ = this->now();
  }
  
  std::string calculateDirection(const Point& from, const Point& to) {
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    
    // esta a priorizar vertical e so depois horizantal
    if (dy > 0) {
      return "UP";
    } else if (dy < 0) {
      return "DOWN";
    } else if (dx > 0) {
      return "RIGHT";
    } else if (dx < 0) {
      return "LEFT";
    }
    
    return "";  // Already at target
  }
  
  bool shouldPick(const Point& target) {// still was not able to test it
    return false; 

    if (holding_box_) {
      return false;
    }
    
    int dist = std::abs(robot_pos_.x - target.x) + std::abs(robot_pos_.y - target.y);
    return dist == 1;
  }
  
  bool shouldDrop(const Point& target) {// still was not able to test it
    return false; 

    if (!holding_box_) {
      return false;
    }
    
    int dist = std::abs(robot_pos_.x - target.x) + std::abs(robot_pos_.y - target.y);
    return dist == 1;
  }
  
  void sendCommand(const std::string& cmd, const Point& target) {
    std_msgs::msg::String cmd_msg;
    cmd_msg.data = cmd;
    cmd_pub_->publish(cmd_msg);
    
    custom_interfaces::msg::RobotControl control_msg;
    control_msg.command = cmd;
    control_msg.target_x = target.x;
    control_msg.target_y = target.y;
    control_pub_->publish(control_msg);
    
    RCLCPP_INFO(this->get_logger(), "Sent command: %s (target: %d, %d)", 
                cmd.c_str(), target.x, target.y);
  }
  
  rclcpp::Subscription<custom_interfaces::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<custom_interfaces::msg::StateEstimationDebug>::SharedPtr position_sub_;
  rclcpp::Subscription<custom_interfaces::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
  rclcpp::Publisher<custom_interfaces::msg::RobotControl>::SharedPtr control_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::vector<Point> path_;
  Point robot_pos_;
  Point expected_pos_;
  bool holding_box_;
  size_t current_waypoint_idx_;
  bool path_received_;
  bool position_received_;
  bool status_received_;
  bool waiting_for_movement_;
  rclcpp::Time command_sent_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}