#pragma once

#include "adapters/estimator_factory.hpp"
#include "adapters/parameters.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "estimators/estimator.hpp"
#include "rclcpp/rclcpp.hpp"
/**
 * @brief Node class for the velocity estimation node.
 *
 * This class is responsible for creating the velocity estimator and creating a publisher.
 * Subclasses of this class are responsible for subscribing to the necessary topics and passing the
 * data to the velocity estimator. The velocity estimator is then used to estimate the vehicle's
 * velocities.
 */
class VENode : public rclcpp::Node {
protected:
  VEParameters _parameters_;
  std::shared_ptr<VelocityEstimator> _velocity_estimator_;
  rclcpp::Publisher<custom_interfaces::msg::Velocities>::SharedPtr _velocities_pub_;
  void publish_velocities() const;

public:
  VENode(const VEParameters& parameters);
};