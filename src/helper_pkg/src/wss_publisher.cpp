#include <iostream>
#include <string>

#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class WheelSpeedsPublisher : public rclcpp::Node {
public:
  WheelSpeedsPublisher() : Node("wheel_speeds_publisher") {
    publisher_ =
        this->create_publisher<eufs_msgs::msg::WheelSpeedsStamped>("/ros_can/wheel_speeds", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&WheelSpeedsPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    std::string input;
    std::cout << "Enter 'p' to publish (or 'q' to quit): ";
    std::getline(std::cin, input);
    if (input == "q") rclcpp::shutdown();
    if (input != "p") return;

    auto message = eufs_msgs::msg::WheelSpeedsStamped();
    // Set the wheel speeds and steering angle here
    message.speeds.lb_speed = 0.0;
    message.speeds.lf_speed = 0.0;
    message.speeds.rb_speed = 0.0;
    message.speeds.rf_speed = 0.0;
    message.speeds.steering = 0.0;
    message.header.stamp = this->now();
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelSpeedsPublisher>());
  rclcpp::shutdown();
  return 0;
}