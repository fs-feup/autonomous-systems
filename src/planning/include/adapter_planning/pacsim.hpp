#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_PACSIM_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_PACSIM_HPP_

#include <empty__struct.hpp>

#include "adapter_planning/adapter.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

class PacSimAdapter : public Adapter {
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pacsim_ebs_server;
  rclcpp::Subscription<pacsim::msg::StampedScalar>::SharedPtr front_steering_sub;
  rclcpp::Subscription<pacsim::msg::StampedScalar>::SharedPtr rear_steering_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pose_sub;

  float provisional_front_steering = 0;
  float provisional_rear_steering = 0;

 public:
  explicit PacSimAdapter(Planning* planning);

  void init() override;
  void mission_state_callback();  // ?
  void set_mission_state(int mission, int state) override;
  void finish() override;
  void front_steering_callback(pacsim::msg::StampedScalar& msg);
  void rear_steering_callback(pacsim::msg::StampedScalar& msg);
  void publish_pose(/*tipo de mensagem do pacsim para posição*/);
};

#endif