#pragma once

#include "common_lib/car_parameters/car_parameters.hpp"

#include <Eigen/Dense>

// Same approach as the one used in load transfer
struct TireInput{
  double vx;
  double vy;
  double wheel_angular_speed;
  double yaw_rate;
  double steering_angle;
  double slip_angle;
  double slip_ratio;
  double vertical_load;
};


/**
 * @brief Class used to model tires. Currently used to model tire forces based on slip.
 *
 */
class TireModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  TireModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}
  /**
   * @brief Calculate the forces acting in a tire based on the tire characteristics and dynamic
   * state.
   *
   * @param tire_input The input parameters for all possible tire models
   * @return Eigen::Vector3d The resulting forces in the tire (Fx, Fy, Mz)
   */
  virtual Eigen::Vector3d tire_forces(const TireInput& tire_input)  = 0;
};
