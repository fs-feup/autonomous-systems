#include "motion_lib/load_transfer_model/base_load_transfer_model.hpp"

class VDLoadTransferModel : public LoadTransferModel {
public:
  VDLoadTransferModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : LoadTransferModel(car_parameters) {}

  /**
   * @brief Computes loads on the tires based on the dynamic state of the vehicle.
   *
   * @param input Receives a pointer to a struct that contains the relevant dynamic state that
   * affects the load transfer.
   * @return Wheels a struct containing the loads on the four tires in Newtons
   */
  common_lib::structures::Wheels compute_loads(const LoadTransferInput& input) const override;

private:
  /**
   * @brief Calculates the total difference in frontal load caused by lateral weight transfer using
   * static load, geometric and elastic.
   *
   * @param massDistribution 0 to 100 value that represents the percentage of weight loaded on the
   * front
   * @param lateral_acceleration Value of lateral acceleration
   * @return float The difference in frontal load caused by the lateral acceleration
   */
  float calculate_front_lateral_transfer(float massDistribution, float lateral_acceleration) const;

  /**
   * @brief Calculates the total difference in rear load caused by lateral weight transfer using
   * static load, geometric and elastic.
   *
   * @param massDistribution 0 to 100 value that represents the percentage of weight loaded on the
   * front
   * @param lateral_acceleration Value of lateral acceleration
   * @return float float The difference in rear load caused by the lateral acceleration
   */
  float calculate_rear_lateral_transfer(float massDistribution, float lateral_acceleration) const;

  /**
   * @brief Calculates the total difference in load caused by longitudinal weight transfer
   *
   * @param longitudinal_acceleration Value of longitudinal acceleration
   * @return float The total difference in load caused by longitudinal acceleration
   */
  float calculate_longitudinal_transfer(float longitudinal_acceleration) const;
};