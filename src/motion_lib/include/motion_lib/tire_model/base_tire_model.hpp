#pragma once

#include <Eigen/Dense>

#include "common_lib/car_parameters/car_parameters.hpp"

enum Tire { FL, FR, RL, RR };

/**
 * @brief Struct used to store all possible input parameters for the tire model. splip angle, ratio,
 * camber and distance to CG will be overwritten by the tire model, no need to fill them.
 *
 */
struct TireInput {
  Tire tire;
  double vx = 0.0;
  double vy = 0.0;
  double yaw_rate = 0.0;
  double steering_angle = 0.0;
  double wheel_angular_speed = 0.0;
  double vertical_load = 0.0;

  double slip_angle = 0.0;
  double slip_ratio = 0.0;
  double distance_to_CG = 0.0;
  double camber_angle = 0.0;
  double vcx = 0.0;  // Longitudinal contact-patch velocity in the tire frame.
  double vcy = 0.0;  // Lateral contact-patch velocity in the tire frame.

  Eigen::Vector4d last_slip_ratio = Eigen::Vector4d::Zero();
  Eigen::Vector4d last_slip_angle = Eigen::Vector4d::Zero();
  double dt = 0;
};

/**
 * @brief Class used to model tires. Currently used to model tire forces based on slip.
 *
 */
class TireModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

  void calculate_slip_angle_front(TireInput& tire_input);
  void calculate_slip_angle_rear(TireInput& tire_input);
  void calculate_slip_ratio(TireInput& tire_input);
  
  void calculate_slip_angle_front_not_transient(TireInput& tire_input);
  void calculate_slip_angle_rear_not_transient(TireInput& tire_input);
  void calculate_slip_ratio_not_transient(TireInput& tire_input);

  /**
   * @brief Calculate the forces acting in a tire based on the tire characteristics and dynamic
   * state.
   *
   * @param tire_input The input parameters for all possible tire models
   * @return Eigen::Vector4d The resulting forces in the tire (Fx, Fy, My, Mz)
   */
  virtual Eigen::Vector4d tire_forces(const TireInput& tire_input) = 0;

public:
  TireModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}

  Eigen::Vector4d calculate_tire_forces(TireInput& tire_input);
  Eigen::Vector4d calculate_tire_forces_not_transient(TireInput& tire_input);
};
