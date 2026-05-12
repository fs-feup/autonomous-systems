#pragma once

#include "models/process/process_model.hpp"
#include "motion_lib/aero_model/map.hpp"
#include "motion_lib/load_transfer_model/map.hpp"
#include "motion_lib/steering_model/map.hpp"
#include "motion_lib/steering_motor_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"
#include "motion_lib/transmission_model/map.hpp"
#include "utils/parameters.hpp"

class RK4VehicleModel : public ProcessModel {
private:
  std::shared_ptr<TransmissionModel> transmission_model_;
  std::shared_ptr<LoadTransferModel> load_transfer_model_;
  std::shared_ptr<AeroModel> aero_model_;
  std::shared_ptr<SteeringModel> steering_model_;
  std::shared_ptr<SteeringMotorModel> steering_motor_model_;
  std::shared_ptr<TireModel> tire_model_;
  Eigen::Matrix<double, StateSize, 1> state_derivative_;

  // Cached parameters for performance
  double half_width_;
  double lf_;
  double lr_;
  double wheel_radius_;
  double inertia_;
  double total_mass_;
  double Izz_;
  double max_peak_torque_;

  // Temporary buffers to avoid repeated allocations
  Eigen::Vector4d wheel_angles_cache_;
  Eigen::Vector4d torques_cache_;
  Eigen::Vector3d aero_forces_cache_;
  Eigen::Vector4d total_vertical_loads_cache_;
  Eigen::Matrix<double, 16, 1> tire_forces_cache_;

  // Helper method to compute common calculations
  void compute_forces_and_moments(const State& state,
                                  common_lib::structures::ControlCommand control_command,
                                  double& total_fx, double& total_fy, double& total_torque);

public:
  RK4VehicleModel(const std::shared_ptr<SEParameters>& parameters);

  void predict(Eigen::Ref<State> state, common_lib::structures::ControlCommand control_command,
               double dt) override;

  Eigen::Matrix<double, StateSize, 1> get_state_derivative(
      Eigen::Ref<State> state, common_lib::structures::ControlCommand control_command);

  VehicleState get_process_model_data(
      const State& state, const common_lib::structures::ControlCommand& control_command) override;
};