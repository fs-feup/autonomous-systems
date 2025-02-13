#pragma once

#include <cmath>
#include <iostream>
#include <utility>

#include "common_lib/maths/transformations.hpp"
#include "motion_lib/car_parameters.hpp"
#include "motion_lib/s2v_model/s2v_model.hpp"

class BicycleModel : public S2VModel {
  common_lib::car_parameters::CarParameters car_parameters_;

public:
  BicycleModel(common_lib::car_parameters::CarParameters car_parameters)
      : car_parameters_(car_parameters) {}

  std::pair<double, double> wheels_velocities_to_cg(double rl_rpm, [[maybe_unused]] double fl_rpm,
                                                    double rr_rpm, [[maybe_unused]] double fr_rpm,
                                                    double steering_angle) override;
  common_lib::structures::Position rear_axis_position(common_lib::structures::Position cg,
                                                      double orientation,
                                                      double dist_cg_2_rear_axis) override;
  Eigen::VectorXd cg_velocity_to_wheels(Eigen::Vector3d& cg_velocities) override;
  Eigen::MatrixXd jacobian_cg_velocity_to_wheels(Eigen::Vector3d& cg_velocities) override;
};