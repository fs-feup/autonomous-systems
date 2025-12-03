#include "common_lib/car_parameters/car_parameters.hpp"

common_lib::car_parameters::CarParameters::CarParameters(double wheel_diameter, double wheelbase,
                                                         double track_width,
                                                         double dist_cg_2_rear_axis,
                                                         double gear_ratio)
    : wheel_diameter(wheel_diameter),
      wheelbase(wheelbase),
      track_width(track_width),
      dist_cg_2_rear_axis(dist_cg_2_rear_axis),
      gear_ratio(gear_ratio) {}

common_lib::car_parameters::CarParameters::CarParameters() {
  std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "global", "global_config");

  YAML::Node global_config = YAML::LoadFile(global_config_path);
  std::string adapter = global_config["global"]["adapter"].as<std::string>();

  std::string car_config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "car", adapter);
  YAML::Node car_config = YAML::LoadFile(car_config_path);
  const auto car = car_config["car"];

  this->wheel_diameter = car["wheel_diameter"].as<double>();
  this->wheelbase = car["wheel_base"].as<double>();
  this->track_width = car["track_width"].as<double>();
  this->dist_cg_2_rear_axis = car["dist_cg_2_rear_axis"].as<double>();
  this->gear_ratio = car["gear_ratio"].as<double>();

  // Dynamic-related parameters (needed by MPC and estimators)
  if (car["cog_height"]) this->cog_height = car["cog_height"].as<double>();
  if (car["mass"]) this->mass = car["mass"].as<double>();
  if (car["powertrainEfficiency"])
    this->powertrainEfficiency = car["powertrainEfficiency"].as<double>();
  if (car["Izz"]) this->Izz = car["Izz"].as<double>();

  // Tire model coefficients (Pacejka-like)
  if (car["tire_B_lateral"])
    this->tire_parameters.tire_B_lateral = car["tire_B_lateral"].as<double>();
  if (car["tire_C_lateral"])
    this->tire_parameters.tire_C_lateral = car["tire_C_lateral"].as<double>();
  if (car["tire_D_lateral"])
    this->tire_parameters.tire_D_lateral = car["tire_D_lateral"].as<double>();
  if (car["tire_E_lateral"])
    this->tire_parameters.tire_E_lateral = car["tire_E_lateral"].as<double>();

  if (car["tire_B_longitudinal"])
    this->tire_parameters.tire_B_longitudinal = car["tire_B_longitudinal"].as<double>();
  if (car["tire_C_longitudinal"])
    this->tire_parameters.tire_C_longitudinal = car["tire_C_longitudinal"].as<double>();
  if (car["tire_D_longitudinal"])
    this->tire_parameters.tire_D_longitudinal = car["tire_D_longitudinal"].as<double>();
  if (car["tire_E_longitudinal"])
    this->tire_parameters.tire_E_longitudinal = car["tire_E_longitudinal"].as<double>();

  // Aero
  if (car["lift_coefficient"])
    this->aero_parameters.lift_coefficient = car["lift_coefficient"].as<double>();
  if (car["drag_coefficient"])
    this->aero_parameters.drag_coefficient = car["drag_coefficient"].as<double>();
  if (car["aero_balance_front"])
    this->aero_parameters.aero_balance_front = car["aero_balance_front"].as<double>();
  if (car["frontal_area"])
    this->aero_parameters.frontal_area = car["frontal_area"].as<double>();

  // Steering limits (optional in car YAML)
  if (car["minimum_steering_angle"])
    this->steering_parameters.minimum_steering_angle = car["minimum_steering_angle"].as<double>();
  if (car["maximum_steering_angle"])
    this->steering_parameters.maximum_steering_angle = car["maximum_steering_angle"].as<double>();
}
