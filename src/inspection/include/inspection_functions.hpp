#ifndef INSPECTION_FUNCTIONS_HPP
#define INSPECTION_FUNCTIONS_HPP

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

class InspectionFunctions {
 public:
  double max_angle;
  double ideal_velocity;
  double turning_period;
  double wheel_radius;
  double gain;
  double finish_time;
  double current_goal_velocity;
  bool start_and_stop;

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