#pragma once

#include <Eigen/Core>
#include <chrono>

#include "motion_lib/aero_model/map.hpp"
#include "motion_lib/battery_model/map.hpp"
#include "motion_lib/load_transfer_model/map.hpp"
#include "motion_lib/motor_model/map.hpp"
#include "motion_lib/steering_model/map.hpp"
#include "motion_lib/steering_motor_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"
#include "motion_lib/transmission_model/map.hpp"
#include "vehicle_model/vehicle_model.hpp"

/**
 * @brief Four-wheel vehicle model with RK4
 */
class RK4StateEstModel : public VehicleModel {
public:
  explicit RK4StateEstModel(const InvictaSimParameters& params);
  ~RK4StateEstModel() override = default;

  void step(double dt, common_lib::structures::Wheels throttle, double angle) override;
  void reset() override;
  std::string get_model_name() const override;

private:
  // Layout of the RK4-integrated state vector.
  enum StateIdx {
    VX = 0,    // body longitudinal velocity [m/s]
    VY,        // body lateral velocity [m/s]
    YAW_RATE,  // yaw rate [rad/s]
    YAW,       // heading [rad]
    PX,        // global x position [m]
    PY,        // global y position [m]
    ST_ANGLE,  // steering angle [rad]
    FL_W,      // wheel angular speeds [rad/s]
    FR_W,
    RL_W,
    RR_W,
    AX,  // longitudinal acceleration state [m/s^2] (relaxes toward vx_dot)
    AY,  // lateral acceleration state [m/s^2] (relaxes toward vy_dot)
    STATE_SIZE
  };
  using StateVec = Eigen::Matrix<double, STATE_SIZE, 1>;

  /**
   * @brief Computes the state derivative for the RK4 integration.
   * @param s Current state vector
   * @param throttle_input Current throttle input (0 to 1)
   * @param steering_target Current steering input (radians)
   * @param dt Time step (s) - provided for any transient dynamics that require it
   * @param write_telemetry Whether to write telemetry for this evaluation (only true for the first RK4 evaluation, k1)
   * @return State derivative vector
   */
  StateVec get_state_derivative(const StateVec& s, double throttle_input, double steering_target,
                                double dt, bool write_telemetry);

  // Sub-models
  std::shared_ptr<TireModel> tire_model_;
  std::shared_ptr<MotorModel> motor_;
  std::shared_ptr<BatteryModel> battery_;
  std::shared_ptr<SteeringMotorModel> steering_motor_;
  std::shared_ptr<TransmissionModel> transmission_;
  std::shared_ptr<AeroModel> aero_;
  std::shared_ptr<LoadTransferModel> load_transfer_;
  std::shared_ptr<SteeringModel> steering_;

  // Cached geometry / inertia (set once in the constructor).
  double lr_ = 0.0;
  double lf_ = 0.0;
  double half_width_ = 0.0;
  double wheel_radius_ = 0.0;
  double inertia_ = 0.0;
  double total_mass_ = 0.0;
  double Izz_ = 0.0;
  double max_peak_torque_ = 0.0;
};
