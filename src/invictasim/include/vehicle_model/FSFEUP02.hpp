#pragma once

#include <chrono>
#include <string>

#include "motion_lib/aero_model/map.hpp"
#include "motion_lib/battery_model/map.hpp"
#include "motion_lib/brake_model/map.hpp"
#include "motion_lib/inverter_model/map.hpp"
#include "motion_lib/load_transfer_model/map.hpp"
#include "motion_lib/motor_model/map.hpp"
#include "motion_lib/steering_model/map.hpp"
#include "motion_lib/steering_motor_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"
#include "motion_lib/transmission_model/map.hpp"
#include "vehicle_model/vehicle_model.hpp"

// State indices for the state vector
enum StateIdx {
  VX = 0,
  VY,
  YAW_RATE,
  YAW,
  PX,
  PY,
  ST_ANGLE,
  FL_W,
  FR_W,
  RL_W,
  RR_W,
  AX,
  AY,
  STATE_SIZE
};

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
  // State vector used for RK4 integration
  using StateVec = Eigen::Matrix<double, STATE_SIZE, 1>;

  // AX/AY are filtered copies of the body accelerations, used to drive the load
  // transfer model.  They must lag the true acceleration by the time the chassis
  // needs to actually roll/pitch onto the springs, not by a second.
  static constexpr double kAccelerationFilterTau = 0.10;  // s

  // Vehicle state struct is defined in the base class
  std::shared_ptr<TireModel> tire_model_;
  std::shared_ptr<MotorModel> motor_;
  std::shared_ptr<BatteryModel> battery_;
  std::shared_ptr<TransmissionModel> transmission_;
  std::shared_ptr<InverterModel> inverter_;
  std::shared_ptr<BrakeModel> brake_;
  std::shared_ptr<AeroModel> aero_;
  std::shared_ptr<LoadTransferModel> load_transfer_;
  std::shared_ptr<SteeringModel> steering_;
  std::shared_ptr<SteeringMotorModel> steering_motor_;
  std::string control_mode_;

  // Calculate the available motor torque based on the throttle input and current motor/battery
  // state
  // One RK4 integration substep.  step() splits the incoming dt into stable
  // substeps (see FSFEUP02.cpp) because the driven-wheel/tyre-slip mode is stiff.
  void integrate_substep(double dt, common_lib::structures::Wheels throttle, double angle);

  double calculate_powertrain_torque(double throttle_input, double dt);

  // Calculate the state derivative for the RK4 integration
  StateVec get_state_derivative(const StateVec& s, double motor_torque,
                                const common_lib::structures::Wheels& brake_torques,
                                double steering_target, double dt, bool write_telemetry,
                                VehicleModelExecutionTimes* execution_times);
};
