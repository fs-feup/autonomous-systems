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
  double max_speed;
  double turning_period;
  double wheel_radius;
  double gain;
  double finish_time;
  bool oscilating;

  /**
   * @brief defines the variables at run time from "config.txt" file
   * 
   * @param path path
   */
  void fromFile(const std::string &path);

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
  double calculate_torque(double velocity);

  /**
   * @brief convert rpm [rotations/minute] to speed [m/s]
   * 
   * @param rpm rotations per minute
   * @return speed [m/s]
   */
  double rpm_to_speed(double rpm);

  /**
   * @brief used when in oscilation mode to redefine the ideal speed
   * 
   * @param current_speed [m/s]
   *
  */
  void redefine_ideal_speed(double current_speed);

  /**
   * @brief Construct a new Inspection Functions object
   * 
   */
  InspectionFunctions();

  /**
   * @brief Construct a new Inspection Mission with constants to be defined by input
   * 
   * @param max_angle maximum turning angle for the wheels in the inspection script [rad]
   * @param ideal_speed ideal speed of the wheels for the inspection script [m/s]
   * @param max_speed maximum speed of the wheels for the mission if oscilating  or the EBS maximum speed [m/s]
   * @param turning_period time it takes for the wheels to turn in inspecion script [s]
   * @param wheel_radius radius of the wheels of the car to estimate speed [m]
   * @param gain constant for the controller in the inspection script
   * @param timer time until the end of the program [s]
   * @param oscilate whether to make the car accelerate and brake multiple times (true) or just once (false)
   */
  InspectionFunctions(double max_angle, double ideal_speed, double max_speed, double turning_period,
  double wheel_radius, double gain, double timer, bool oscilate);
};