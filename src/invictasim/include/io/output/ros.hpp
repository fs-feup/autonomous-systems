#pragma once

#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "io/output/output_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

/**
 * @brief ROS-based simulator output adapter.
 */
class RosOutputAdapter : public rclcpp::Node, public InvictaSimOutputAdapter {
public:
  /**
   * @brief Construct a new RosOutputAdapter.
   * @param simulator Simulator instance.
   */
  explicit RosOutputAdapter(const std::shared_ptr<InvictaSim>& simulator);

  /**
   * @brief Publish the current simulator outputs to ROS topics.
   * @param vehicle_model Vehicle model with the current state.
   * @param steering_angle Current steering command.
   */
  void publish_outputs(const std::shared_ptr<VehicleModel>& vehicle_model,
                       double steering_angle) override;

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      motor_torque_pub_;  ///< Publisher for motor torque.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      battery_current_pub_;  ///< Publisher for battery current.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      battery_voltage_pub_;  ///< Publisher for battery voltage.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      battery_soc_pub_;  ///< Publisher for battery state of charge.
  rclcpp::Publisher<custom_interfaces::msg::VehicleStateVector>::SharedPtr
      vehicle_state_pub_;  ///< Publisher for the vehicle state message.
};
