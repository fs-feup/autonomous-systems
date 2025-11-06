// Discrete control to follow planned path and publish UP/DOWN/LEFT/RIGHT

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_interfaces/msg/path.hpp>
#include <custom_interfaces/msg/state_estimation_position.hpp>

#include <vector>
#include <optional>
#include <cmath>

class ControlNode : public rclcpp::Node {
public:
  ControlNode() : Node("control") {
    using std::placeholders::_1;
    pos_sub_ = this->create_subscription<custom_interfaces::msg::StateEstimationPosition>(
      "state_estimation/position", 10, std::bind(&ControlNode::onPosition, this, _1));
    path_sub_ = this->create_subscription<custom_interfaces::msg::Path>(
      "planning/path", 10, std::bind(&ControlNode::onPath, this, _1));
    cmd_pub_ = this->create_publisher<std_msgs::msg::String>("control/cmd", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&ControlNode::onTimer, this));
  }

private:
  void onPosition(const custom_interfaces::msg::StateEstimationPosition::SharedPtr msg) {
    rx_ = msg->x; ry_ = msg->y; have_pos_ = true;
  }
  void onPath(const custom_interfaces::msg::Path::SharedPtr msg) {
    waypoints_ = msg->waypoints; target_idx_ = 0;
  }
  void onTimer() {
    if (!have_pos_ || target_idx_ >= static_cast<int>(waypoints_.size())) return;
    double tx = waypoints_[target_idx_].x;
    double ty = waypoints_[target_idx_].y;
    double dx = std::round(tx - rx_);
    double dy = std::round(ty - ry_);
    std_msgs::msg::String cmd;
    if (std::abs(dx) > std::abs(dy)) {
      cmd.data = (dx > 0) ? "RIGHT" : "LEFT";
    } else if (std::abs(dy) > 0) {
      cmd.data = (dy > 0) ? "UP" : "DOWN";
    } else {
      target_idx_++;
      return;
    }
    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<custom_interfaces::msg::StateEstimationPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool have_pos_{false};
  double rx_{0.0}, ry_{0.0};
  std::vector<geometry_msgs::msg::Point> waypoints_;
  int target_idx_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
