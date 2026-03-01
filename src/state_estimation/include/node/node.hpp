#pragma once

#include <std_msgs/msg/float64.hpp>

#include "config/parameters.hpp"
#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "estimators/map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/parameters.hpp"
#include "utils/state_define.hpp"
/**
 * @brief Node class for the state estimation module.
 *
 * This class is responsible for creating the state estimator and creating a publisher.
 * Subclasses of this class are responsible for subscribing to the necessary topics and passing the
 * data to the state estimator. The state estimator is then used to estimate the vehicle's
 * state.
 */
class SENode : public rclcpp::Node {
protected:
  SEParameters _params_;
  std::shared_ptr<StateEstimator> _state_estimator_;

  rclcpp::TimerBase::SharedPtr _timer_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _execution_time_pub_;
  rclcpp::Publisher<custom_interfaces::msg::VehicleStateVector>::SharedPtr _state_pub_;

  void publish_state(const State& state, const rclcpp::Time time);

  void timer_callback();

public:
  SENode(const SEParameters& parameters);
};