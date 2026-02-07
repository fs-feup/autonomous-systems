#pragma once

#include "motion_lib/battery_model/map.hpp"
#include "motion_lib/differential_model/map.hpp"
#include "motion_lib/motor_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"
#include "vehicle_model/vehicle_model.hpp"

/**
 * @brief Four wheel vehicle model (tuned for FSFEUP02)
 */
class FSFEUP02Model : public VehicleModel {
public:
  explicit FSFEUP02Model(const InvictaSimParameters& params);
  ~FSFEUP02Model() override = default;

  void step(double dt, double angle, double throttle) override;
  void reset() override;

  double get_position_x() const override;
  double get_position_y() const override;
  double get_yaw() const override;
  double get_velocity_x() const override;

  void set_position(double x, double y, double yaw) override;
  void set_velocity(double vx) override;

  std::string get_model_name() const override;

private:
  // Still need all the state variables, we need to define what the state of the vehicle model will
  // be
  std::shared_ptr<TireModel> front_left;
  std::shared_ptr<TireModel> front_right;
  std::shared_ptr<TireModel> rear_left;
  std::shared_ptr<TireModel> rear_right;

  std::shared_ptr<MotorModel> motor_;
  std::shared_ptr<BatteryModel> battery_;
  std::shared_ptr<DifferentialModel> differential_;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double vx_ = 0.0;
};
