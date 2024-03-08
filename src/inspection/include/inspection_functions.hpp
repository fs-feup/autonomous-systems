#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "file_utils/file.hpp"

class InspectionFunctions {
 public:
  double max_angle;
  double ideal_speed;
  double turning_period;
  double wheel_radius;
  double gain;
  double max_speed;

  /**
   * @brief defines the variables at run time
   * 
   * @param path path
   */
  void define_constants(const std::string &path);

  /**
   * @brief get the steering angle according to time
   * 
   * @param time in seconds
   * @return steering angle in radians
   */
  double calculate_steering(double time);

  /**
   * @brief calculate torque according to velocity
   * 
   * @param velocity 
   * @return double 
   */
  double calculate_torque(double velocity);

  /**
   * @brief convert rpm to speed in meters per second
   * 
   * @param rpm rotations per minute
   * @return speed in meters per second
   */
  double rpm_to_speed(double rpm);

  /**
   * @brief Construct a new Inspection Functions object
   * 
   */
  InspectionFunctions();
};