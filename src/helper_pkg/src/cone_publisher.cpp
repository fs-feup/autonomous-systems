#include <iostream>
#include <string>

#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "rclcpp/rclcpp.hpp"

class ConePublisher : public rclcpp::Node {
public:
  ConePublisher() : Node("cone_publisher") {
    publisher_ = this->create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>("cones", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&ConePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = eufs_msgs::msg::ConeArrayWithCovariance();
    std::string input;
    while (true) {
      std::cout << "Enter X coordinate (or 'q' to quit): ";
      std::getline(std::cin, input);
      if (input == "q") break;
      double x = std::stod(input);

      std::cout << "Enter Y coordinate (or 'q' to quit): ";
      std::getline(std::cin, input);
      if (input == "q") break;
      double y = std::stod(input);

      auto cone = eufs_msgs::msg::ConeWithCovariance();
      cone.point.x = x;
      cone.point.y = y;
      message.big_orange_cones.push_back(cone);
    }
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConePublisher>());
  rclcpp::shutdown();
  return 0;
}