#include <pacsim/msg/stamped_scalar.hpp>
#include <pacsim/msg/wheels.hpp>
#include <rclcpp/rclcpp.hpp>

#include "custom_interfaces/msg/control_command.hpp"

class ControlBridgeNode : public rclcpp::Node {
public:
  explicit ControlBridgeNode();
  void controlCallback(const custom_interfaces::msg::ControlCommand msg);

private:
  rclcpp::Subscription<custom_interfaces::msg::ControlCommand>::SharedPtr control_sub_;
  rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr acceleration_pub_;
  rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steering_pub_;
};