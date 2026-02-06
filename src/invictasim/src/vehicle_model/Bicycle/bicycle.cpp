#include "vehicle_model/Bicycle/bicycle.hpp"

BicycleModel::BicycleModel(const std::string& config_path) {
  YAML::Node config = YAML::LoadFile(config_path);

  auto model_config = config["vehicle_model"];

  // Load kinematics parameters
  lr_ = model_config["kinematics"]["lr"].as<double>();
  lf_ = model_config["kinematics"]["lf"].as<double>();
  sf_ = model_config["kinematics"]["sf"].as<double>();
  sr_ = model_config["kinematics"]["sr"].as<double>();
  h_cg_ = model_config["kinematics"]["h_cg"].as<double>();
  max_steering_angle_ =
      model_config["kinematics"]["max_steering_angle"].as<double>() * M_PI / 180.0;

  // Load tire parameters
  Blat_ = model_config["tire"]["Blat"].as<double>();
  Clat_ = model_config["tire"]["Clat"].as<double>();
  Dlat_ = model_config["tire"]["Dlat"].as<double>();
  Elat_ = model_config["tire"]["Elat"].as<double>();

  // Load aerodynamics
  cla_ = model_config["aero"]["cla"].as<double>();
  cda_ = model_config["aero"]["cda"].as<double>();
  aero_area_ = model_config["aero"]["aeroArea"].as<double>();

  // Load mass and inertia
  mass_ = model_config["m"].as<double>();
  Izz_ = model_config["Izz"].as<double>();

  // Load drivetrain parameters
  wheel_radius_ = model_config["wheelRadius"].as<double>();
  gear_ratio_ = model_config["gearRatio"].as<double>();

  // Initialize state variables
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;
  vx_ = 0.0;
  steering_angle_ = 0.0;
  throttle_ = 0.0;
}

void BicycleModel::step(double dt) {
  (void)dt;
  // Some Logic, Bicycle model class would have a MotorModel, TireModel, etc and it will
  // call them here or something similar
}

void BicycleModel::reset() {
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;
  vx_ = 0.0;
  steering_angle_ = 0.0;
  throttle_ = 0.0;
}

void BicycleModel::set_position(double x, double y, double yaw) {
  x_ = x;
  y_ = y;
  yaw_ = yaw;
}

void BicycleModel::set_velocity(double vx) { vx_ = vx; }

double BicycleModel::get_position_x() const { return x_; }

double BicycleModel::get_position_y() const { return y_; }

double BicycleModel::get_yaw() const { return yaw_; }

double BicycleModel::get_velocity_x() const { return vx_; }

void BicycleModel::set_steering(double angle) { steering_angle_ = angle; }

void BicycleModel::set_throttle(double throttle) { throttle_ = throttle; }

std::string BicycleModel::get_model_name() const { return "BicycleModel"; }
