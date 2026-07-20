#pragma once

#include "aero_parameters.hpp"
#include "battery_parameters.hpp"
#include "brake_parameters.hpp"
#include "common_lib/config_load/config_load.hpp"
#include "inverter_parameters.hpp"
#include "common_lib/structures/physical_constants.hpp"
#include "common_lib/structures/wheels.hpp"
#include "load_transfer_parameters.hpp"
#include "motor_parameters.hpp"
#include "steering_motor_parameters.hpp"
#include "steering_parameters.hpp"
#include "tire_parameters.hpp"
#include "transmission_parameters.hpp"

namespace common_lib::car_parameters {
struct CarParameters {
  double front_bearing_drag;
  double wheel_diameter;
  double wheelbase;
  double track_width;
  double cg_2_rear_axis;
  double gear_ratio;
  double cg_height;
  double sprung_mass;
  double unsprung_mass;
  double total_mass;
  double sprung_cg_y;
  double sprung_cg_z;
  double unsprung_cg_y;
  double unsprung_cg_z;
  double Izz;
  double ackerman_factor;
  double front_wheel_inertia; 
  double rear_wheel_inertia;

  std::shared_ptr<common_lib::car_parameters::TireParameters> tire_parameters;
  std::shared_ptr<common_lib::car_parameters::AeroParameters> aero_parameters;
  std::shared_ptr<common_lib::car_parameters::SteeringMotorParameters> steering_motor_parameters;
  std::shared_ptr<common_lib::car_parameters::SteeringParameters> steering_parameters;
  std::shared_ptr<common_lib::car_parameters::LoadTransferParameters> load_transfer_parameters;
  std::shared_ptr<common_lib::car_parameters::MotorParameters> motor_parameters;
  std::shared_ptr<common_lib::car_parameters::BatteryParameters> battery_parameters;
  std::shared_ptr<common_lib::car_parameters::TransmissionParameters> transmission_parameters;
  std::shared_ptr<common_lib::car_parameters::BrakeParameters> brake_parameters;
  std::shared_ptr<common_lib::car_parameters::InverterParameters> inverter_parameters;
  std::shared_ptr<common_lib::structures::PhysicalConstants> physical_constants;
  std::shared_ptr<common_lib::structures::Wheels> wheels;

  CarParameters();
  CarParameters(std::string config_dir, std::string config_name);
};

}  // namespace common_lib::car_parameters
