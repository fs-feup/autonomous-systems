#include "vehicle_model/FSFEUP02/FSFEUP02.hpp"

FSFEUP02Model::FSFEUP02Model(const std::string& config_path) {
  YAML::Node config = YAML::LoadFile(config_path);

  auto model_config = config["vehicle_model"];

  // Load kinematics parameters
  // lr_ = model_config["kinematics"]["lr"].as<double>();
  // lf_ = model_config["kinematics"]["lf"].as<double>();
  // sf_ = model_config["kinematics"]["sf"].as<double>();
  // sr_ = model_config["kinematics"]["sr"].as<double>();
  // h_cg_ = model_config["kinematics"]["h_cg"].as<double>();
  // max_steering_angle_ =
  //     model_config["kinematics"]["max_steering_angle"].as<double>() * M_PI / 180.0;

  // IMPORTANT: LOAD THE TIRE CONFIGURATION BASED ON THE TIRE FIELD

  // Load tire parameters
  // float effective_wheel_radius = model_config["tire"]["effective_tire_radius"].as<float>();
  float d_bleft = model_config["tire"]["d_bleft"].as<float>();
  float d_bright = model_config["tire"]["d_bright"].as<float>();
  float d_fleft = model_config["tire"]["d_fleft"].as<float>();
  float d_fright = model_config["tire"]["d_fright"].as<float>();
  float camber = model_config["tire"]["camber"].as<float>();

  // Initialize tires
  front_left = std::make_unique<TireModel>(camber, d_fleft);
  front_right = std::make_unique<TireModel>(camber, d_fright);
  back_left = std::make_unique<TireModel>(camber, d_bleft);
  back_right = std::make_unique<TireModel>(camber, d_bright);

  // Load aerodynamics
  // cla_ = model_config["aero"]["cla"].as<double>();
  // cda_ = model_config["aero"]["cda"].as<double>();
  // aero_area_ = model_config["aero"]["aeroArea"].as<double>();

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

void FSFEUP02Model::step(double dt) {
  (void)dt;
  // Some Logic, Bicycle model class would have a MotorModel, TireModel, etc and it will
  // call them here or something similar
}

void FSFEUP02Model::reset() {
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;
  vx_ = 0.0;
  steering_angle_ = 0.0;
  throttle_ = 0.0;
}

void FSFEUP02Model::set_position(double x, double y, double yaw) {
  x_ = x;
  y_ = y;
  yaw_ = yaw;
}

void FSFEUP02Model::set_velocity(double vx) { vx_ = vx; }

double FSFEUP02Model::get_position_x() const { return x_; }

double FSFEUP02Model::get_position_y() const { return y_; }

double FSFEUP02Model::get_yaw() const { return yaw_; }

double FSFEUP02Model::get_velocity_x() const { return vx_; }

void FSFEUP02Model::set_steering(double angle) { steering_angle_ = angle; }

void FSFEUP02Model::set_throttle(double throttle) { throttle_ = throttle; }

std::string FSFEUP02Model::get_model_name() const { return "FSFEUP02Model"; }

float FSFEUP02Model::get_tire_effective_radius() const {
  return front_left->get_tire_effective_radius();
}
