#include "motion_lib/load_transfer_model/base_load_transfer_model.hpp"

class RigidBodyLoadTransferModel : public LoadTransferModel {
public:
  RigidBodyLoadTransferModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : LoadTransferModel(car_parameters) {}

  /**
   * @brief Computes loads on the tires based on the dynamic state of the vehicle.
   *
   * @param input Receives a pointer to a struct that contains the relevant dynamic state that
   * affects the load transfer.
   * @return Wheels a struct containing the loads on the four tires in Newtons
   */
  common_lib::structures::Wheels compute_loads(const LoadTransferInput& input) const override;
};