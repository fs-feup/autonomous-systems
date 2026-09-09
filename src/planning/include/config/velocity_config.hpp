#pragma once

#include <vector>

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
   * @brief Maximum lateral acceleration.
   */
  double lateral_acceleration_;

  /**
   * @brief Maximum longitudinal acceleration.
   */
  double longitudinal_acceleration_;

  /**
   * @brief Flag to enable/disable velocity planning.
   */
  bool use_velocity_planning_;

  /**
   * @brief Flag to enable/disable adaptive velocity planning.
   */
  bool use_adaptive_velocity_;

  /**
   * @brief Minimum curvature value for a point to be considered a corner apex / section boundary.
   */
  double curvature_peak_threshold_;

  /**
   * @brief Minimum number of path points between two section boundaries (prevents
   * over-segmentation).
   */
  int min_section_spacing_;

  /**
   * @brief Mean error anchors for adaptive delta calculations.
   */
  std::vector<double> adaptive_anchor_mean_;

  /**
   * @brief Delta limit anchors corresponding to the mean error anchors.
   */
  std::vector<double> adaptive_anchor_delta_;

  /**
   * @brief Default constructor.
   */
  VelocityPlanningConfig()
      : minimum_velocity_(3.0),
        desired_velocity_(5.0),
        braking_acceleration_(-4.0),
        lateral_acceleration_(7.0),
        longitudinal_acceleration_(7.0),
        use_velocity_planning_(true),
        use_adaptive_velocity_(true),
        curvature_peak_threshold_(0.05),
        min_section_spacing_(5),
        adaptive_anchor_mean_({0.00, 0.05, 0.10, 0.15, 0.20, 0.30, 0.90, 1.00, 1.50}),
        adaptive_anchor_delta_({2.00, 1.50, 1.00, 0.85, 0.65, -0.20, -1.00, -1.25, -1.50}) {}

  /**
   * @brief Parameterized constructor.
   */
  VelocityPlanningConfig(double minimum_velocity, double desired_velocity,
                         double braking_acceleration, double lateral_acceleration,
                         double longitudinal_acceleration, bool use_velocity_planning,
                         bool use_adaptive_velocity, double curvature_peak_threshold,
                         int min_section_spacing, const std::vector<double>& adaptive_anchor_mean,
                         const std::vector<double>& adaptive_anchor_delta)
      : minimum_velocity_(minimum_velocity),
        desired_velocity_(desired_velocity),
        braking_acceleration_(braking_acceleration),
        lateral_acceleration_(lateral_acceleration),
        longitudinal_acceleration_(longitudinal_acceleration),
        use_velocity_planning_(use_velocity_planning),
        use_adaptive_velocity_(use_adaptive_velocity),
        curvature_peak_threshold_(curvature_peak_threshold),
        min_section_spacing_(min_section_spacing),
        adaptive_anchor_mean_(adaptive_anchor_mean),
        adaptive_anchor_delta_(adaptive_anchor_delta) {}
};