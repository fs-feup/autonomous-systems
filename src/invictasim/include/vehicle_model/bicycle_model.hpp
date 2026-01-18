#pragma once

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "vehicle_model/vehicle_model.hpp"

/**
 * @brief Simple bicycle model structure
 */
class BicycleModel : public VehicleModel {
public:
  explicit BicycleModel(const std::string& config_path);
  ~BicycleModel() override = default;
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

private:
  // Kinematics parameters
  double lr_;    // Distance from CG to rear axle
  double lf_;    // Distance from CG to front axle
  double sf_;    // Track width front
  double sr_;    // Track width rear
  double h_cg_;  // Height of center of gravity
  double max_steering_angle_;

  // Tire parameters
  double Blat_;
  double Clat_;
  double Dlat_;
  double Elat_;

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
};
