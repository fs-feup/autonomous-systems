#include "motion_lib/load_transfer_model/base_load_transfer_model.hpp"

class RigidBodyLoadTransferModel : public LoadTransferModel {
public:
  RigidBodyLoadTransferModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : LoadTransferModel(car_parameters) {}

  /**
   * @brief Computes loads on the tires based on the dynamic state of the vehicle.
   *
   * @param dynamic_state Receives only the lateral and longitudinal accelerations
   * @return Eigen::Vector4d a vector containing the loads on the four tires in Newtons, in the
   * order: FL, FR, RL, RR.
   */
  Eigen::Vector4d compute_loads(const Eigen::VectorXd& dynamic_state) const override;
};