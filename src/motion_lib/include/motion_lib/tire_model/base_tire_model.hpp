#pragma once

#include <Eigen/Dense>

#include "common_lib/car_parameters/car_parameters.hpp"

enum Tire { FL, FR, RL, RR };

// Same approach as the one used in load transfer
/**
 * @brief Struct used to store all possible input parameters for the tire model. splip angle, ratio,
 * camber and distance to CG will be overwritten by the tire model, no need to fill them.
 *
 */
struct TireInput {
  Tire tire;
  double vx;
  double vy;
  double yaw_rate;
  double steering_angle;
  double wheel_angular_speed;
  double vertical_load;

  double slip_angle;
  double slip_ratio;
  double distance_to_CG;
  double camber_angle;

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
<<<<<<< HEAD
  void calculate_slip_angle_front_not_transient(TireInput& tire_input);
  void calculate_slip_angle_rear(TireInput& tire_input);
  void calculate_slip_angle_rear_not_transient(TireInput& tire_input);
  void calculate_slip_ratio(TireInput& tire_input);
  void calculate_slip_ratio_not_transient(TireInput& tire_input);
=======
  void calculate_slip_angle_rear(TireInput& tire_input);
  void calculate_slip_ratio(TireInput& tire_input);
>>>>>>> main

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
<<<<<<< HEAD
  Eigen::Vector4d calculate_tire_forces_not_transient(TireInput& tire_input);
=======
>>>>>>> main
};
