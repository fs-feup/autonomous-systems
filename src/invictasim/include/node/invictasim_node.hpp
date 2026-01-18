#pragma once

#include <chrono>
#include <memory>
#include <thread>

#include "config/config_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
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
  double sim_time_;
  std::chrono::steady_clock::time_point next_loop_time_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr test_pub_;

  // Subscribers

  /**
   * @brief Initialize the simulator node
   */
  void init();

  /**
   * @brief Perform a single simulation step
   */
  void simulation_step();
};
