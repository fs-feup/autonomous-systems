#pragma once

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "vd_helpers.hpp"
#include "vehicle_model/tire_model/tire_model.hpp"
#include "vehicle_model/vehicle_model.hpp"

/**
 * @brief Four wheel vehicle model (tuned for FSFEUP02)
 */
class FSFEUP02Model : public VehicleModel {
public:
  explicit FSFEUP02Model(const std::string& config_path);
  ~FSFEUP02Model() override = default;
  void step(double dt) override;
  void reset() override;

  double get_position_x() const override;
  double get_position_y() const override;
  double get_yaw() const override;
  double get_velocity_x() const override;

  void set_position(double x, double y, double yaw) override;
  void set_velocity(double vx) override;
  void set_steering(double angle) override;
  void set_throttle(double throttle) override;

  std::string get_model_name() const override;
  float get_tire_effective_radius() const;

  // CHANGE PARAMETERS ARE STILL FROM BICYCLE MODEL
private:
  VehicleModelParams params;
  VehicleModelState state;
  // Aerodynamics
  double cla_;
  double cda_;
  double aero_area_;

  // Mass and inertia
  double mass_;
  double Izz_;

  // Drivetrain
  double wheel_radius_;
  double gear_ratio_;

  // State variables
  double x_;
  double y_;
  double yaw_;
  double vx_;

  // Control inputs
  double steering_angle_;
  double throttle_;

  // Tires
  std::unique_ptr<TireModel> front_left;
  std::unique_ptr<TireModel> front_right;
  std::unique_ptr<TireModel> back_left;
  std::unique_ptr<TireModel> back_right;
};
