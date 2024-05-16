#pragma once

#include "adapter_control/adapter.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"

class Control;

class PacSimAdapter : public Adapter {
 private:
  rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr steering_pub;
  rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr acceleration_pub;

  // TODO: MISSION FINISHED IS A SERVICE NOT A TOPIC,

 public:
  explicit PacSimAdapter(Control* control);
  virtual void finish() override;
  virtual void publish_cmd(float acceleration = 0, float steering = 0) override;
};