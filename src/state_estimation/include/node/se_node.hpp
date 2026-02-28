#pragma once

#include <std_msgs/msg/float64.hpp>

#include "config/parameters.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "estimators/map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/parameters.hpp"
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
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _execution_time_pub_;

  void publish_state();

public:
  SENode(const SEParameters& parameters);
};