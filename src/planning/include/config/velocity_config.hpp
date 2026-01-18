#pragma once

/**
 * @brief Configuration parameters for the Velocity Planning class.
 */
struct VelocityPlanningConfig {
  /**
   * @brief Minimum speed in the velocity planning.
   */
  double minimum_velocity_;

  /**
   * @brief The desired velocity of the car.
   */
  double desired_velocity_;

  /**
   * @brief Maximum braking acceleration.
   */
  double braking_acceleration_;

  /**
   * @brief Maximum forward acceleration.
   */
  double acceleration_;

  /**
   * @brief The maximum normal acceleration.
   */
  double normal_acceleration_;

  /**
   * @brief Flag to enable/disable velocity planning.
   */
  bool use_velocity_planning_;

  /**
   * @brief Default constructor.
   */
  VelocityPlanningConfig()
      : minimum_velocity_(3.0),
        desired_velocity_(5.0),
        braking_acceleration_(-4.0),
        acceleration_(7.0),
        normal_acceleration_(7.0),
        use_velocity_planning_(true) {}

  /**
   * @brief Parameterized constructor.
   */
  VelocityPlanningConfig(double minimum_velocity, double desired_velocity,
                         double braking_acceleration, double acceleration,
                         double normal_acceleration, bool use_velocity_planning)
      : minimum_velocity_(minimum_velocity),
        desired_velocity_(desired_velocity),
        braking_acceleration_(braking_acceleration),
        acceleration_(acceleration),
        normal_acceleration_(normal_acceleration),
        use_velocity_planning_(use_velocity_planning) {}
};