#pragma once

#include "adapter_control/adapter.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"

class PacSimAdapter : public Adapter {
 private:
  rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr steering_pub_;
  rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr acceleration_pub_;

  // TODO: MISSION FINISHED IS A SERVICE NOT A TOPIC,

 public:
  explicit PacSimAdapter(Control* control);
  void publish_cmd(double acceleration = 0, double steering = 0) override;
};