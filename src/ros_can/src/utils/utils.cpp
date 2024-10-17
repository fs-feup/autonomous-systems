//
// Created by promao on 5/5/24.
//
#include "utils/utils.hpp"

/**
 * @brief Creates buffer from steering angle for Cubemars
 * actuator position command
 *
 * @param angle steering angle in radians
 * @param buffer steering angle in buffer in degrees * 10000
 */
void create_steering_angle_command(float angle, char* buffer) {
  float degree_angle = angle * 180 / M_PI;
  int converted_angle = static_cast<int>((degree_angle * 10000));  // Indicated by documentation
  for (unsigned int i = 0; i < 4; i++) {
    buffer[i] = converted_angle >> (8 * (3 - i));
  }
}

/**
 * @brief Converts the steering angle command from wheels to
 * actuator position command
 *
 * @param double wheels_steering_angle steering angle in radians at the wheels
 * @param double actuator_steering_angle steering angle in radians for the actuator
 * @return int returns 0 if successful, 1 if error

*/
int transform_steering_angle_command(const double wheels_steering_angle,
                                     double& actuator_steering_angle) {
  // WARNING: -------- Código feito pelo António Guedes - PERIGO --------

  // Calculation for beta_1
  double input_beta_1 = (70 - 71.333 * cos(0.0965167 + wheels_steering_angle)) / 335.48;
  if (input_beta_1 < -1 || input_beta_1 > 1) {
    RCLCPP_ERROR(rclcpp::get_logger("ros_can"),
                 "Error: Input to asin for beta_1 is out of range: %f", input_beta_1);
    return 1;
  }
  double beta_1 = asin(input_beta_1);
  double delta_x_1 =
      71.333 * sin(0.0965167 + wheels_steering_angle) + 335.48 * cos(beta_1) - 349.164;

  // Calculation for beta_2
  double input_beta_2 = (70 - 71.333 * cos(0.0965167 - wheels_steering_angle)) / 335.48;
  if (input_beta_2 < -1 || input_beta_2 > 1) {
    RCLCPP_ERROR(rclcpp::get_logger("ros_can"),
                 "Error: Input to asin for beta_2 is out of range: %f", input_beta_2);
    return 1;
  }
  double beta_2 = asin(input_beta_2);
  double delta_x_2 =
      71.333 * sin(0.0965167 - wheels_steering_angle) + 335.48 * cos(beta_2) - 349.164;

  double delta_x_average = (delta_x_1 - delta_x_2) / 2;
  double alpha = delta_x_average * (2 * M_PI / 87.9);  // Ensure floating point division

  actuator_steering_angle = alpha;  // Set angle
  return 0;
}

/**
 * @brief Converts the steering angle from sensor to wheel angle
 *
 * @param double sensor_steering_angle steering angle in degrees from the sensor
 * @param double wheels_steering_angle steering angle in radians at the wheels
 * @return int returns 0 if successful, 1 if error
*/
int transform_steering_angle_reading(const double sensor_steering_angle, double
                                     &wheels_steering_angle) {

    double m = 0.203820182;
    double b = 5.1114e-16;

    wheels_steering_angle = (m * sensor_steering_angle + b);
    return 0;
}
