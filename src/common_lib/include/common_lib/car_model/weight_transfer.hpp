#pragma once

#include <Eigen/Dense>

#include "common_lib/car_parameters/car_parameters.hpp"

namespace common_lib::car_model {

/**
 * @brief Calculate the weight transfer based on car parameters, on accelerations and aerodynamic
 * forces.
 *
 * @param accelerations accelerations at the center of gravity in the x and y directions
 * (longitudinal and lateral)
 * @param aero_forces aerodynamic forces in the x, y, and z directions
 * @param car_params car parameters including mass, wheelbase, center of gravity height, etc.
 * @return Eigen::Vector4d load on the wheels
 *         in the order of [FL, FR, RL, RR].
 */
Eigen::Vector4d weight_transfer(const Eigen::Vector2d accelerations,
                                const Eigen::Vector3d& aero_forces,
                                const common_lib::car_parameters::CarParameters& car_params);

}  // namespace common_lib::car_model