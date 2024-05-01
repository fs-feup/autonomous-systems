#ifndef INSPECTION_FUNCTIONS_HPP
#define INSPECTION_FUNCTIONS_HPP

#define VELOCITY_ERROR_FOR_MAX_THROTTLE (10.0)

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

class InspectionFunctions {
 public:
  // default to EBS test values
  double max_angle = 0.52359877559;
  double ideal_velocity = 2.0;
  double turning_period = 4.0;
  double wheel_radius = 0.254;
  double gain = 0.25;
  double finish_time = 26.0;
  double current_goal_velocity = 2.0;
  bool start_and_stop = false;

  /**
   * @brief calculate the steering angle according to time
   *
   * @param time in seconds
   * @return steering angle in radians
   */
  double calculate_steering(double time);

  /**
   * @brief calculate torque output according to velocity
   *
   * @param velocity
   * @return double
   */
  double calculate_throttle(double current_velocity);

  /**
   * @brief convert rpm [rotations/minute] to velocity [m/s]
   *
   * @param rpm rotations per minute
   * @return velocity [m/s]
   */
  double rpm_to_velocity(double rpm);

  /**
   * @brief used when in oscilation mode to redefine the ideal velocity
   *
   * @param current_velocity [m/s]
   *
   */
  void redefine_goal_velocity(double current_velocity);

  /**
   * @brief convert the throttle to a range between -1 and 1
   *
   * @param throttle original throttle value
   * @return double throttle value between -1 and 1
   */
  double throttle_to_adequate_range(double throttle);

  /**
   * @brief Construct a new Inspection Functions object
   *
   */
  InspectionFunctions();

  /**
   * @brief Construct a new Inspection Mission with constants to be defined by input
   *
   * @param max_angle maximum turning angle for the wheels in the inspection script [rad]
   * @param ebs_test_ideal_velocity ideal velocity of the wheels for the ebs test [m/s]
   * @param inspection_ideal_velocity ideal velocity of the wheels for the inspection [m/s]
   * @param turning_period time it takes for the wheels to turn in inspecion script [s]
   * @param wheel_radius radius of the wheels of the car to estimate velocity [m]
   * @param inspection_gain constant for the controller in the inspection
   * @param ebs_test_gain constant for the controller in the ebs test
   * @param finish_time time until the end of the program [s]
   * @param start_and_stop whether to make the car accelerate and brake multiple times (true) or
   * just once (false)
   */
  InspectionFunctions(double max_angle, double turning_period, double wheel_radius,
                      double finish_time, bool start_and_stop, double gain = 0.0,
                      double ideal_velocity = 0.0);
};

#endif  // INSPECTION_FUNCTIONS_HPP