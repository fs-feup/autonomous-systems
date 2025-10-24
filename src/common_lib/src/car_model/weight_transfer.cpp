#include "common_lib/car_model/weight_transfer.hpp"

namespace common_lib::car_model {

/**/
Eigen::Vector4d weight_transfer(const Eigen::Vector2d accelerations,
                                const Eigen::Vector3d& aero_forces,
                                const common_lib::car_parameters::CarParameters& car_params) {
  Eigen::Vector4d weight_transfer;
  weight_transfer.setZero();

  return weight_transfer;
}

}  // namespace common_lib::car_model