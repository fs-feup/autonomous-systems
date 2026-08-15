#include "motion_lib/aero_model/map_based_aero_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

common_lib::car_parameters::CarParameters make_test_parameters(double ride_height_front,
                                                               double ride_height_rear) {
  common_lib::car_parameters::CarParameters params("invictasim/vehicle_models", "03");
  params.aero_parameters->ride_height_front = ride_height_front;
  params.aero_parameters->ride_height_rear = ride_height_rear;
  return params;
}

double recover_cd(const MapBasedAeroModel& model,
                  const common_lib::car_parameters::AeroParameters& aero) {
  const double vx = 10.0;
  const Eigen::Vector3d forces = model.aero_forces(Eigen::Vector3d(vx, 0.0, 0.0));
  return -2.0 * forces[0] / (aero.air_density * aero.frontal_area * std::abs(vx) * vx);
}

double recover_cl(const MapBasedAeroModel& model,
                  const common_lib::car_parameters::AeroParameters& aero) {
  const double vx = 10.0;
  const Eigen::Vector3d forces = model.aero_forces(Eigen::Vector3d(vx, 0.0, 0.0));
  return -2.0 * forces[2] / (aero.air_density * aero.frontal_area * vx * vx);
}

}  // namespace

TEST(MapBasedAeroModel, UsesExactGridPointCoefficients) {
  auto params = make_test_parameters(103.75, 20.0);
  MapBasedAeroModel model(params);

  EXPECT_NEAR(recover_cd(model, *params.aero_parameters), 1.42957, 1e-9);
  EXPECT_NEAR(recover_cl(model, *params.aero_parameters), -3.72157, 1e-9);
}

TEST(MapBasedAeroModel, InterpolatesBetweenRideHeightFrontRows) {
  auto params = make_test_parameters(107.5, 20.0);
  MapBasedAeroModel model(params);

  const double expected_cd = (1.42957 + 1.40469) / 2.0;

  EXPECT_NEAR(recover_cd(model, *params.aero_parameters), expected_cd, 1e-9);
}
