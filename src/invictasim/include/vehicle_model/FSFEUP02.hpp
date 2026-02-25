#pragma once

#include "motion_lib/aero_model/map.hpp"
#include "motion_lib/battery_model/map.hpp"
#include "motion_lib/differential_model/map.hpp"
#include "motion_lib/load_transfer_model/map.hpp"
#include "motion_lib/motor_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"
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

  /**
   * @brief Get the current motor torque being applied
   */
  double get_motor_torque() const override;

  /**
   * @brief Get the current battery current draw
   */
  double get_battery_current() const override;

  /**
   * @brief Get the current battery voltage
   */
  double get_battery_voltage() const override;

  /**
   * @brief Get the current battery state of charge
   */
  double get_battery_soc() const override;

private:
  // Vehicle state struct is defined in the base class
  std::shared_ptr<TireModel> front_left;
  std::shared_ptr<TireModel> front_right;
  std::shared_ptr<TireModel> rear_left;
  std::shared_ptr<TireModel> rear_right;

  std::shared_ptr<MotorModel> motor_;
  std::shared_ptr<BatteryModel> battery_;
  std::shared_ptr<DifferentialModel> differential_;
  std::shared_ptr<AeroModel> aero_;
  std::shared_ptr<LoadTransferModel> load_transfer_;

  // Helper function to calculate the torque combining the motor model and the battery model
  double calculate_powertrain_torque(double throttle_input, double dt);

// Tire functions
  Eigen::Vector3d calculateTireForces(std::string tire_name, double load);
  // Helper function to calculate the slip angle on each tire of the front axle
  // A separation is needed because the front axle needs to account for steering angle (FWD)
  double calculateSlipAngleFront(double dist_to_cg , bool isLeft);
  double calculateSlipAngleRear(double dist_to_cg , bool isLeft);
  double calculateSlipRatio(double wheel_angular_velocity);

};
