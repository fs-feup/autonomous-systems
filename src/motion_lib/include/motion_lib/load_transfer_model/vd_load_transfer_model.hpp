#include "motion_lib/load_transfer_model/base_load_transfer_model.hpp"

class VDLoadTransferModel : public LoadTransferModel {
public:
  VDLoadTransferModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : LoadTransferModel(car_parameters) {}

  /**
   * @brief Computes loads on the tires based on the dynamic state of the vehicle.
   *
   * @param dynamic_state Receives a vector containing the following values in this order: [longitudinal_acceleration, lateral_acceleration, downforce]
   * @return Eigen::Vector4d a vector containing the loads on the four tires in Newtons, in the
   * order: FL, FR, RL, RR.
   */
  Eigen::Vector4d compute_loads(const Eigen::VectorXd& dynamic_state) const override;

  /**
   * @brief Calculates the total difference in frontal load caused by lateral weight transfer using static load, geometric and elastic.
   * 
   * @param massDistribution 0 to 100 value that represents the percentage of weight loaded on the front
   * @param lateral_acceleration Value of lateral acceleration 
   * @return float The difference in frontal load caused by the lateral acceleration
   */
  float calculate_front_lateral_transfer(float massDistribution , float lateral_acceleration) const ;

  /**
   * @brief Calculates the total difference in rear load caused by lateral weight transfer using static load, geometric and elastic.
   * 
   * @param massDistribution 0 to 100 value that represents the percentage of weight loaded on the front
   * @param lateral_acceleration Value of lateral acceleration 
   * @return float float The difference in rear load caused by the lateral acceleration
   */
  float calculate_rear_lateral_transfer(float massDistribution , float lateral_acceleration) const ;

  /**
   * @brief Calculates the total difference in load caused by longitudinal weight transfer
   * 
   * @param longitudinal_acceleration Value of longitudinal acceleration
   * @return float The total difference in load caused by longitudinal acceleration
   */
  float calculate_longitudinal_transfer(float longitudinal_acceleration) const;
};