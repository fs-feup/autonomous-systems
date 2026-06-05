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
 * @brief Four-wheel vehicle model (StateEstModel physics) advanced with RK4.
 *
 * Same dynamics as StateEstModel, but integrated with a classic 4th-order Runge-Kutta
 * scheme structured like the RK4 process model in state_estimation: a pure
 * get_state_derivative() evaluated four times per step. Heading and global position are
 * integrated inside the RK4 as well (an improvement over the state-estimation version,
 * which only integrates the dynamic states and leaves pose to a separate step).
 *
 * NOTE: this requires the *non-transient* tire model. RK4 evaluates the derivative at
 * intermediate stages that are not the real state; a transient tire model carries
 * internal slip state and would be corrupted by those off-trajectory evaluations. The
 * non-transient model is memoryless, so it is safe to call repeatedly per step.
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
   * @brief Continuous-time state derivative at `s`.
   *
   * The accelerations AX/AY are themselves integrated states (their derivative is
   * vx_dot - ax, like the state-estimation model), and the load transfer reads those
   * states, breaking the algebraic accel<->load loop. When `write_telemetry` is true the
   * computed forces/loads/slips are written straight to `state_` for publishing; only the
   * start-of-step (k1) stage sets it, so the off-trajectory RK4 stages don't clobber it.
   *
   * `dt` is used only to stabilize the stiff wheel-spin mode at low speed (its slip
   * stiffness is folded into the effective wheel inertia, a linearly-implicit treatment);
   * it does not otherwise affect the derivative and vanishes at speed.
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
