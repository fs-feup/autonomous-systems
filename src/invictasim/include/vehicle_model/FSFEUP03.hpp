#pragma once

#include <chrono>

#include "motion_lib/aero_model/map.hpp"
#include "motion_lib/battery_model/map.hpp"
#include "motion_lib/brake_model/map.hpp"
#include "motion_lib/inverter_model/map.hpp"
#include "motion_lib/load_transfer_model/map.hpp"
#include "motion_lib/motor_model/map.hpp"
#include "motion_lib/steering_model/map.hpp"
#include "motion_lib/steering_motor_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"
#include "vehicle_model/vehicle_model.hpp"
#include "motion_lib/independent_drive_model/base_independent_drive_model.hpp"
#include "motion_lib/independent_drive_model/map.hpp"

/**
 * @brief Four wheel vehicle model (tuned for FSFEUP03)
 */
class FSFEUP03Model : public VehicleModel {
public:
  /**
   * @brief Construct a new FSFEUP03Model object
   */
  explicit FSFEUP03Model(const InvictaSimParameters& params);

  /**
   * @brief Destroy the FSFEUP03Model object
   */
  ~FSFEUP03Model() override = default;

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
  std::shared_ptr<MotorModel> motor_left_;
  std::shared_ptr<MotorModel> motor_right_;
  std::shared_ptr<IndependentDriveModel> drive_left_;
  std::shared_ptr<IndependentDriveModel> drive_right_;
  std::shared_ptr<BatteryModel> battery_;
  std::shared_ptr<TransmissionModel> transmission_;
  std::shared_ptr<InverterModel> inverter_;
  std::shared_ptr<BrakeModel> brake_;
  std::shared_ptr<AeroModel> aero_;
  std::shared_ptr<LoadTransferModel> load_transfer_;
  std::shared_ptr<SteeringModel> steering_;
  std::shared_ptr<SteeringMotorModel> steering_motor_;
  std::string control_mode_;

  // Helper function to calculate the torque combining the motor model and the battery model
  std::pair<double, double> calculate_side_powertrain(
    double throttle_input, double wheel_speed,
    const std::shared_ptr<MotorModel>& motor,
    const std::shared_ptr<IndependentDriveModel>& drive);
};
