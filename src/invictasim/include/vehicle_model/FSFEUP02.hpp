#pragma once

#include <chrono>

#include "motion_lib/aero_model/map.hpp"
#include "motion_lib/battery_model/map.hpp"
#include "motion_lib/load_transfer_model/map.hpp"
#include "motion_lib/motor_model/map.hpp"
#include "motion_lib/steering_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"
#include "motion_lib/transmission_model/map.hpp"
#include "vehicle_model/vehicle_model.hpp"

/**
 * @brief Four wheel vehicle model (tuned for FSFEUP02)
 */
class FSFEUP02Model : public VehicleModel {
public:
  /**
   * @brief Construct a new FSFEUP02Model object
   */
  explicit FSFEUP02Model(const InvictaSimParameters& params);

  /**
   * @brief Destroy the FSFEUP02Model object
   */
  ~FSFEUP02Model() override = default;

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
  std::shared_ptr<TransmissionModel> transmission_;
  std::shared_ptr<AeroModel> aero_;
  std::shared_ptr<LoadTransferModel> load_transfer_;
  std::shared_ptr<SteeringModel> steering_;

  // Helper function to calculate the torque combining the motor model and the battery model
  double calculate_powertrain_torque(double throttle_input, double dt);
};
