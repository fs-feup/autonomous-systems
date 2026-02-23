#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <memory>
#include <thread>

#include "config/config.hpp"
#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "vehicle_model/map.hpp"
#include "vehicle_model/vehicle_model.hpp"

class InvictaSimNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor
   */
  explicit InvictaSimNode(const InvictaSimParameters& params);

  /**
   * @brief Destructor
   */
  ~InvictaSimNode() = default;

private:
  InvictaSimParameters params_;

  // Vehicle model
  std::shared_ptr<VehicleModel> vehicle_model_;

  // Simulation state
  common_lib::structures::Wheels throttle_input_;
  double steering_input_;
  double sim_time_;
  std::chrono::steady_clock::time_point next_loop_time_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_soc_pub_;

  rclcpp::Publisher<custom_interfaces::msg::VehicleStateVector>::SharedPtr vehicle_state_pub_;

  // Subscribers
  rclcpp::Subscription<custom_interfaces::msg::ControlCommand>::SharedPtr control_command_sub_;

  /**
   * @brief Initialize the simulator node
   */
  void init();

  /**
   * @brief Perform a single simulation step
   */
  void simulation_step();
};
