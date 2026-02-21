#pragma once

#include "models/process/process_model.hpp"
#include "motion_lib/aero_model/map.hpp"
#include "motion_lib/load_transfer_model/map.hpp"
#include "motion_lib/steering_model/map.hpp"
#include "motion_lib/steering_motor_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"
#include "utils/parameters.hpp"

class ComponentBasedVehicleModel : public ProcessModel {
private:
  std::shared_ptr<AeroModel> aero_model_;
  std::shared_ptr<LoadTransferModel> load_transfer_model_;
  std::shared_ptr<SteeringModel> steering_model_;
  std::shared_ptr<SteeringMotorModel> steering_motor_model_;
  std::shared_ptr<TireModel> tire_model_;
  SEParameters parameters_;

public:
  ComponentBasedVehicleModel(SEParameters parameters);

  void predict(Eigen::Matrix<double, 10, 1>& state, Eigen::Matrix<double, 10, 10>& covariance,
               const Eigen::Matrix<double, 10, 10>& process_noise_matrix,
               common_lib::structures::ControlCommand control_command, double dt) override;
};