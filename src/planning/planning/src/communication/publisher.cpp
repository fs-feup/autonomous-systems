#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/msg/point2d.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node
{
  public:
    Publisher()
    : Node("planning_endurance"), count_(0)
    {
      publisher_ = this->create_publisher<custom_interfaces::msg::Point2d>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Publisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = custom_interfaces::msg::Point2d();
      message.x = 1;
      message.y = 2;
      RCLCPP_INFO(this->get_logger(), "Publishing: x = %f | y = %f", message.x, message.y);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_interfaces::msg::Point2d>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
