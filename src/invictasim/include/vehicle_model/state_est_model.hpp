#pragma once

#include <chrono>

#include "motion_lib/aero_model/map.hpp"
#include "motion_lib/battery_model/map.hpp"
#include "motion_lib/differential_model/map.hpp"
#include "motion_lib/load_transfer_model/map.hpp"
#include "motion_lib/motor_model/map.hpp"
#include "motion_lib/steering_model/map.hpp"
#include "motion_lib/steering_motor_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"
#include "vehicle_model/vehicle_model.hpp"

/**
 * @brief Four wheel vehicle model (used in State estimation)
 */
class StateEstModel : public VehicleModel {
public:
  /**
   * @brief Construct a new StateEstModel object
   */
  explicit StateEstModel(const InvictaSimParameters& params);

  /**
   * @brief Destroy the StateEstModel object
   */
  ~StateEstModel() override = default;

  /**
   * @brief Step the vehicle model forward in time based on the current state and the control inputs
   * (steering angle and throttle)
   */
  void step(double dt, common_lib::structures::Wheels throttle, double angle) override;

  /**
   * @brief Reset the vehicle state to the initial conditions
   */
  void reset() override;

  /**
   * @brief Get the model name
   */
  std::string get_model_name() const override;

private:
  // Vehicle state struct is defined in the base class
  std::shared_ptr<TireModel> tire_model_;
  std::shared_ptr<MotorModel> motor_;
  std::shared_ptr<BatteryModel> battery_;
  std::shared_ptr<SteeringMotorModel> steering_motor_;
  std::shared_ptr<DifferentialModel> differential_;
  std::shared_ptr<AeroModel> aero_;
  std::shared_ptr<LoadTransferModel> load_transfer_;
  std::shared_ptr<SteeringModel> steering_;

  // Helper function to calculate wheel-drive torque with motor and battery limits.
  double calculate_powertrain_torque(double throttle_input, double dt);
};
