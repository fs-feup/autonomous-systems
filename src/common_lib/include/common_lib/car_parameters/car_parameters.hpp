#pragma once

#include "aero_parameters.hpp"
#include "battery_parameters.hpp"
#include "common_lib/config_load/config_load.hpp"
#include "differential_parameters.hpp"
#include "load_transfer_parameters.hpp"
#include "motor_parameters.hpp"
#include "steering_motor_parameters.hpp"
#include "steering_parameters.hpp"
#include "tire_parameters.hpp"

namespace common_lib::car_parameters {
struct CarParameters {
  double wheel_diameter;
  double wheelbase;
  double track_width;
  double dist_cg_2_rear_axis;
  double gear_ratio;
  double cog_height;
  double mass;
  double powertrainEfficiency;
  double Izz;

  std::shared_ptr<common_lib::car_parameters::TireParameters> tire_parameters;
  std::shared_ptr<common_lib::car_parameters::AeroParameters> aero_parameters;
  std::shared_ptr<common_lib::car_parameters::SteeringMotorParameters> steering_motor_parameters;
  std::shared_ptr<common_lib::car_parameters::SteeringParameters> steering_parameters;
  std::shared_ptr<common_lib::car_parameters::LoadTransferParameters> load_transfer_parameters;
  std::shared_ptr<common_lib::car_parameters::MotorParameters> motor_parameters;
  std::shared_ptr<common_lib::car_parameters::BatteryParameters> battery_parameters;
  std::shared_ptr<common_lib::car_parameters::DifferentialParameters> differential_parameters;

  CarParameters();
  CarParameters(std::string config_dir, std::string config_name);
};

}  // namespace common_lib::car_parameters