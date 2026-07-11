#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <map>
#include <chrono>
#include <random>
#include <algorithm>
#include <future>
#include <filesystem>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/structures/wheels.hpp"
#include "config/config.hpp"

#include "motion_lib/aero_model/map.hpp"
#include "motion_lib/battery_model/map.hpp"
#include "motion_lib/brake_model/map.hpp"
#include "motion_lib/inverter_model/map.hpp"
#include "motion_lib/load_transfer_model/map.hpp"
#include "motion_lib/motor_model/map.hpp"
#include "motion_lib/steering_model/map.hpp"
#include "motion_lib/steering_motor_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"
#include "motion_lib/transmission_model/map.hpp"


// Constants
constexpr double kEpsilon = 1e-6;

struct CsvRow {
  double timestamp_s;
  double throttle_fl;
  double throttle_fr;
  double throttle_rl;
  double throttle_rr;
  double steering;
  double real_x;
  double real_y;
  double real_yaw;
  double real_vx;
  double real_vy;
  double real_yaw_rate;
  double real_fl_rpm;
  double real_fr_rpm;
  double real_rl_rpm;
  double real_rr_rpm;
  double real_motor_rpm;
};

struct SimState {
  double vx = 0.0;
  double vy = 0.0;
  double yaw_rate = 0.0;
  double yaw = 0.0;
  double x = 0.0;
  double y = 0.0;
  double steering_angle = 0.0;
  double wl_fl = 0.0;
  double wl_fr = 0.0;
  double wl_rl = 0.0;
  double wl_rr = 0.0;
  double ax = 0.0;
  double ay = 0.0;

  double motor_torque = 0.0;
  double motor_omega = 0.0;
  double motor_current = 0.0;
  double battery_current = 0.0;
  double battery_voltage = 0.0;

  Eigen::Vector4d last_slip_ratio = Eigen::Vector4d::Zero();
  Eigen::Vector4d last_slip_angle = Eigen::Vector4d::Zero();

  common_lib::structures::Wheels wheels_torque;
  common_lib::structures::Wheels wheels_vertical_load;

  double aero_drag = 0.0;
  double aero_downforce = 0.0;
};

using StateVec = Eigen::Matrix<double, 13, 1>;

struct RunningRmse {
  double sum_squares = 0.0;
  int count = 0;

  void update(double value) {
    sum_squares += value * value;
    count++;
  }

  double get() const {
    if (count <= 0) return 0.0;
    return std::sqrt(sum_squares / static_cast<double>(count));
  }
};

struct ParameterSpec {
  std::string name;
  double min_val;
  double max_val;
};

struct Individual {
  std::vector<double> values;
  double score = 1e9;
};

struct PoseSample {
  double x;
  double y;
  double yaw;
};

double normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

PoseSample transform_pose_to_map(
    const PoseSample& pose, const PoseSample& source_origin, const PoseSample& target_origin) {
  const double dx = pose.x - source_origin.x;
  const double dy = pose.y - source_origin.y;
  const double source_cos = std::cos(source_origin.yaw);
  const double source_sin = std::sin(source_origin.yaw);
  const double local_x = source_cos * dx + source_sin * dy;
  const double local_y = -source_sin * dx + source_cos * dy;
  const double local_yaw = normalize_angle(pose.yaw - source_origin.yaw);

  const double target_cos = std::cos(target_origin.yaw);
  const double target_sin = std::sin(target_origin.yaw);
  return {target_origin.x + target_cos * local_x - target_sin * local_y,
          target_origin.y + target_sin * local_x + target_cos * local_y,
          normalize_angle(target_origin.yaw + local_yaw)};
}

std::vector<CsvRow> read_csv(const std::string& path) {
  std::vector<CsvRow> rows;
  std::ifstream file(path);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open CSV file: " << path << std::endl;
    return rows;
  }

  std::string line;
  if (!std::getline(file, line)) {
    return rows;
  }

  std::vector<std::string> headers;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    while (!cell.empty() && (cell.back() == '\r' || cell.back() == ' ')) cell.pop_back();
    headers.push_back(cell);
  }

  auto get_col_idx = [&](const std::string& name) -> int {
    for (size_t i = 0; i < headers.size(); ++i) {
      if (headers[i] == name) return i;
    }
    return -1;
  };

  int idx_time = get_col_idx("timestamp_s");
  int idx_throttle_fl = get_col_idx("throttle_fl");
  int idx_throttle_fr = get_col_idx("throttle_fr");
  int idx_throttle_rl = get_col_idx("throttle_rl");
  int idx_throttle_rr = get_col_idx("throttle_rr");
  int idx_steering = get_col_idx("steering");
  int idx_x = get_col_idx("real_x");
  int idx_y = get_col_idx("real_y");
  int idx_yaw = get_col_idx("real_yaw");
  int idx_vx = get_col_idx("real_vx");
  int idx_vy = get_col_idx("real_vy");
  int idx_yaw_rate = get_col_idx("real_yaw_rate");
  int idx_fl_rpm = get_col_idx("real_fl_rpm");
  int idx_fr_rpm = get_col_idx("real_fr_rpm");
  int idx_rl_rpm = get_col_idx("real_rl_rpm");
  int idx_rr_rpm = get_col_idx("real_rr_rpm");
  int idx_motor_rpm = get_col_idx("real_motor_rpm");

  if (idx_time < 0 || idx_steering < 0 || idx_x < 0 || idx_y < 0) {
    std::cerr << "Error: CSV is missing required columns in " << path << std::endl;
    return rows;
  }

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    std::vector<std::string> values;
    std::stringstream line_ss(line);
    while (std::getline(line_ss, cell, ',')) {
      values.push_back(cell);
    }

    if (values.size() < headers.size()) continue;

    auto get_val = [&](int idx, double default_val = 0.0) -> double {
      if (idx < 0 || idx >= static_cast<int>(values.size())) return default_val;
      try {
        return std::stod(values[idx]);
      } catch (...) {
        return default_val;
      }
    };

    CsvRow row;
    row.timestamp_s = get_val(idx_time);
    row.throttle_fl = get_val(idx_throttle_fl);
    row.throttle_fr = get_val(idx_throttle_fr);
    row.throttle_rl = get_val(idx_throttle_rl);
    row.throttle_rr = get_val(idx_throttle_rr);
    row.steering = get_val(idx_steering);
    row.real_x = get_val(idx_x);
    row.real_y = get_val(idx_y);
    row.real_yaw = get_val(idx_yaw);
    row.real_vx = get_val(idx_vx);
    row.real_vy = get_val(idx_vy);
    row.real_yaw_rate = get_val(idx_yaw_rate);
    row.real_fl_rpm = get_val(idx_fl_rpm);
    row.real_fr_rpm = get_val(idx_fr_rpm);
    row.real_rl_rpm = get_val(idx_rl_rpm);
    row.real_rr_rpm = get_val(idx_rr_rpm);
    row.real_motor_rpm = get_val(idx_motor_rpm);

    rows.push_back(row);
  }

  return rows;
}

// Reflection-like map of parameters. Add any new parameters you want to tune here!
std::map<std::string, double*> get_parameter_ptrs(std::shared_ptr<common_lib::car_parameters::CarParameters> p) {
  std::map<std::string, double*> m;
  if (!p) return m;

  m["car.front_bearing_drag"] = &p->front_bearing_drag;
  m["car.wheel_diameter"] = &p->wheel_diameter;
  m["car.wheelbase"] = &p->wheelbase;
  m["car.track_width"] = &p->track_width;
  m["car.cg_2_rear_axis"] = &p->cg_2_rear_axis;
  m["car.gear_ratio"] = &p->gear_ratio;
  m["car.cg_height"] = &p->cg_height;
  m["car.sprung_mass"] = &p->sprung_mass;
  m["car.unsprung_mass"] = &p->unsprung_mass;
  m["car.total_mass"] = &p->total_mass;
  m["car.sprung_cg_y"] = &p->sprung_cg_y;
  m["car.sprung_cg_z"] = &p->sprung_cg_z;
  m["car.unsprung_cg_y"] = &p->unsprung_cg_y;
  m["car.unsprung_cg_z"] = &p->unsprung_cg_z;
  m["car.Izz"] = &p->Izz;
  m["car.ackerman_factor"] = &p->ackerman_factor;

  if (p->aero_parameters) {
    m["aero.lift_coefficient"] = &p->aero_parameters->lift_coefficient;
    m["aero.drag_coefficient"] = &p->aero_parameters->drag_coefficient;
    m["aero.aero_side_force_coefficient"] = &p->aero_parameters->aero_side_force_coefficient;
    m["aero.aero_balance_front"] = &p->aero_parameters->aero_balance_front;
    m["aero.frontal_area"] = &p->aero_parameters->frontal_area;
    m["aero.air_density"] = &p->aero_parameters->air_density;
  }
  if (p->battery_parameters) {
    m["battery.capacity_ah"] = &p->battery_parameters->capacity_ah;
    m["battery.initial_soc"] = &p->battery_parameters->initial_soc;
    m["battery.max_discharge_current"] = &p->battery_parameters->max_discharge_current;
    m["battery.max_charge_current"] = &p->battery_parameters->max_charge_current;
    m["battery.min_voltage"] = &p->battery_parameters->min_voltage;
    m["battery.min_soc"] = &p->battery_parameters->min_soc;
    m["battery.OCV_a5"] = &p->battery_parameters->OCV_a5;
    m["battery.OCV_a4"] = &p->battery_parameters->OCV_a4;
    m["battery.OCV_a3"] = &p->battery_parameters->OCV_a3;
    m["battery.OCV_a2"] = &p->battery_parameters->OCV_a2;
    m["battery.OCV_a1"] = &p->battery_parameters->OCV_a1;
    m["battery.OCV_a0"] = &p->battery_parameters->OCV_a0;
    m["battery.R0_a5"] = &p->battery_parameters->R0_a5;
    m["battery.R0_a4"] = &p->battery_parameters->R0_a4;
    m["battery.R0_a3"] = &p->battery_parameters->R0_a3;
    m["battery.R0_a2"] = &p->battery_parameters->R0_a2;
    m["battery.R0_a1"] = &p->battery_parameters->R0_a1;
    m["battery.R0_a0"] = &p->battery_parameters->R0_a0;
    m["battery.R1_a5"] = &p->battery_parameters->R1_a5;
    m["battery.R1_a4"] = &p->battery_parameters->R1_a4;
    m["battery.R1_a3"] = &p->battery_parameters->R1_a3;
    m["battery.R1_a2"] = &p->battery_parameters->R1_a2;
    m["battery.R1_a1"] = &p->battery_parameters->R1_a1;
    m["battery.R1_a0"] = &p->battery_parameters->R1_a0;
    m["battery.C1_a5"] = &p->battery_parameters->C1_a5;
    m["battery.C1_a4"] = &p->battery_parameters->C1_a4;
    m["battery.C1_a3"] = &p->battery_parameters->C1_a3;
    m["battery.C1_a2"] = &p->battery_parameters->C1_a2;
    m["battery.C1_a1"] = &p->battery_parameters->C1_a1;
    m["battery.C1_a0"] = &p->battery_parameters->C1_a0;
  }
  if (p->brake_parameters) {
    m["brake.max_front_torque"] = &p->brake_parameters->max_front_torque;
    m["brake.max_rear_torque"] = &p->brake_parameters->max_rear_torque;
  }
  if (p->inverter_parameters) {
    m["inverter.efficiency"] = &p->inverter_parameters->efficiency;
    m["inverter.max_phase_current"] = &p->inverter_parameters->max_phase_current;
    m["inverter.acceleration_delay_ms"] = &p->inverter_parameters->acceleration_delay_ms;
    m["inverter.coast_delay_ms"] = &p->inverter_parameters->coast_delay_ms;
    m["inverter.regen_braking_delay_ms"] = &p->inverter_parameters->regen_braking_delay_ms;
  }
  if (p->load_transfer_parameters) {
    m["load_transfer.roll_axis_z"] = &p->load_transfer_parameters->roll_axis_z;
    m["load_transfer.front_roll_center_z"] = &p->load_transfer_parameters->front_roll_center_z;
    m["load_transfer.rear_roll_center_z"] = &p->load_transfer_parameters->rear_roll_center_z;
    m["load_transfer.front_stiffness_distribution"] = &p->load_transfer_parameters->front_stiffness_distribution;
    m["load_transfer.pitch_center_z"] = &p->load_transfer_parameters->pitch_center_z;
  }
  if (p->motor_parameters) {
    m["motor.max_peak_rpm"] = &p->motor_parameters->max_peak_rpm;
    m["motor.max_continuous_rpm"] = &p->motor_parameters->max_continuous_rpm;
    m["motor.max_peak_current"] = &p->motor_parameters->max_peak_current;
    m["motor.max_continuous_current"] = &p->motor_parameters->max_continuous_current;
    m["motor.max_continuous_power"] = &p->motor_parameters->max_continuous_power;
    m["motor.max_peak_power"] = &p->motor_parameters->max_peak_power;
    m["motor.max_continuous_torque"] = &p->motor_parameters->max_continuous_torque;
    m["motor.max_peak_torque"] = &p->motor_parameters->max_peak_torque;
    m["motor.kt_constant"] = &p->motor_parameters->kt_constant;
    m["motor.peak_duration"] = &p->motor_parameters->peak_duration;
    m["motor.fade_start"] = &p->motor_parameters->fade_start;
  }
  if (p->steering_motor_parameters) {
    m["steering_motor.kp"] = &p->steering_motor_parameters->kp;
    m["steering_motor.ki"] = &p->steering_motor_parameters->ki;
    m["steering_motor.kd"] = &p->steering_motor_parameters->kd;
    m["steering_motor.time_constant"] = &p->steering_motor_parameters->time_constant;
  }
  if (p->steering_parameters) {
    m["steering.minimum_steering_angle"] = &p->steering_parameters->minimum_steering_angle;
    m["steering.maximum_steering_angle"] = &p->steering_parameters->maximum_steering_angle;
    m["steering.ackerman_factor"] = &p->steering_parameters->ackerman_factor;
  }
  if (p->tire_parameters) {
    m["tire.tire_B_lateral"] = &p->tire_parameters->tire_B_lateral;
    m["tire.tire_C_lateral"] = &p->tire_parameters->tire_C_lateral;
    m["tire.tire_D_lateral"] = &p->tire_parameters->tire_D_lateral;
    m["tire.tire_E_lateral"] = &p->tire_parameters->tire_E_lateral;
    m["tire.tire_B_longitudinal"] = &p->tire_parameters->tire_B_longitudinal;
    m["tire.tire_C_longitudinal"] = &p->tire_parameters->tire_C_longitudinal;
    m["tire.tire_D_longitudinal"] = &p->tire_parameters->tire_D_longitudinal;
    m["tire.tire_E_longitudinal"] = &p->tire_parameters->tire_E_longitudinal;
    m["tire.camber_scaling_factor"] = &p->tire_parameters->camber_scaling_factor;
    m["tire.effective_tire_r"] = &p->tire_parameters->effective_tire_r;
    m["tire.fr_toe"] = &p->tire_parameters->fr_toe;
    m["tire.fl_toe"] = &p->tire_parameters->fl_toe;
    m["tire.rr_toe"] = &p->tire_parameters->rr_toe;
    m["tire.rl_toe"] = &p->tire_parameters->rl_toe;
    m["tire.wheel_inertia"] = &p->tire_parameters->wheel_inertia;
    m["tire.slip_angle_relaxation_length"] = &p->tire_parameters->slip_angle_relaxation_length;
    m["tire.slip_ratio_relaxation_length"] = &p->tire_parameters->slip_ratio_relaxation_length;
    m["tire.d_bright"] = &p->tire_parameters->d_bright;
    m["tire.d_bleft"] = &p->tire_parameters->d_bleft;
    m["tire.d_fright"] = &p->tire_parameters->d_fright;
    m["tire.d_fleft"] = &p->tire_parameters->d_fleft;
    m["tire.fr_camber"] = &p->tire_parameters->fr_camber;
    m["tire.fl_camber"] = &p->tire_parameters->fl_camber;
    m["tire.rr_camber"] = &p->tire_parameters->rr_camber;
    m["tire.rl_camber"] = &p->tire_parameters->rl_camber;
    m["tire.UNLOADED_RADIUS"] = &p->tire_parameters->UNLOADED_RADIUS;
    m["tire.WIDTH"] = &p->tire_parameters->WIDTH;
    m["tire.ASPECT_RATIO"] = &p->tire_parameters->ASPECT_RATIO;
    m["tire.RIM_RADIUS"] = &p->tire_parameters->RIM_RADIUS;
    m["tire.RIM_WIDTH"] = &p->tire_parameters->RIM_WIDTH;
    m["tire.INFLPRES"] = &p->tire_parameters->INFLPRES;
    m["tire.NOMPRES"] = &p->tire_parameters->NOMPRES;
    m["tire.MASS"] = &p->tire_parameters->MASS;
    m["tire.IXX"] = &p->tire_parameters->IXX;
    m["tire.IYY"] = &p->tire_parameters->IYY;
    m["tire.BELT_MASS"] = &p->tire_parameters->BELT_MASS;
    m["tire.BELT_IXX"] = &p->tire_parameters->BELT_IXX;
    m["tire.BELT_IYY"] = &p->tire_parameters->BELT_IYY;
    m["tire.GRAVITY"] = &p->tire_parameters->GRAVITY;
    m["tire.FNOMIN"] = &p->tire_parameters->FNOMIN;
    m["tire.VERTICAL_STIFFNESS"] = &p->tire_parameters->VERTICAL_STIFFNESS;
    m["tire.VERTICAL_DAMPING"] = &p->tire_parameters->VERTICAL_DAMPING;
    m["tire.MC_CONTOUR_A"] = &p->tire_parameters->MC_CONTOUR_A;
    m["tire.MC_CONTOUR_B"] = &p->tire_parameters->MC_CONTOUR_B;
    m["tire.BREFF"] = &p->tire_parameters->BREFF;
    m["tire.DREFF"] = &p->tire_parameters->DREFF;
    m["tire.FREFF"] = &p->tire_parameters->FREFF;
    m["tire.Q_RE0"] = &p->tire_parameters->Q_RE0;
    m["tire.Q_V1"] = &p->tire_parameters->Q_V1;
    m["tire.Q_V2"] = &p->tire_parameters->Q_V2;
    m["tire.Q_FZ2"] = &p->tire_parameters->Q_FZ2;
    m["tire.Q_FCX"] = &p->tire_parameters->Q_FCX;
    m["tire.Q_FCY"] = &p->tire_parameters->Q_FCY;
    m["tire.Q_CAM"] = &p->tire_parameters->Q_CAM;
    m["tire.PFZ1"] = &p->tire_parameters->PFZ1;
    m["tire.Q_FCY2"] = &p->tire_parameters->Q_FCY2;
    m["tire.Q_CAM1"] = &p->tire_parameters->Q_CAM1;
    m["tire.Q_CAM2"] = &p->tire_parameters->Q_CAM2;
    m["tire.Q_CAM3"] = &p->tire_parameters->Q_CAM3;
    m["tire.Q_FYS1"] = &p->tire_parameters->Q_FYS1;
    m["tire.Q_FYS2"] = &p->tire_parameters->Q_FYS2;
    m["tire.Q_FYS3"] = &p->tire_parameters->Q_FYS3;
    m["tire.BOTTOM_OFFST"] = &p->tire_parameters->BOTTOM_OFFST;
    m["tire.BOTTOM_STIFF"] = &p->tire_parameters->BOTTOM_STIFF;
    m["tire.LONGITUDINAL_STIFFNESS"] = &p->tire_parameters->LONGITUDINAL_STIFFNESS;
    m["tire.LATERAL_STIFFNESS"] = &p->tire_parameters->LATERAL_STIFFNESS;
    m["tire.YAW_STIFFNESS"] = &p->tire_parameters->YAW_STIFFNESS;
    m["tire.FREQ_LONG"] = &p->tire_parameters->FREQ_LONG;
    m["tire.FREQ_LAT"] = &p->tire_parameters->FREQ_LAT;
    m["tire.FREQ_YAW"] = &p->tire_parameters->FREQ_YAW;
    m["tire.FREQ_WINDUP"] = &p->tire_parameters->FREQ_WINDUP;
    m["tire.DAMP_LONG"] = &p->tire_parameters->DAMP_LONG;
    m["tire.DAMP_LAT"] = &p->tire_parameters->DAMP_LAT;
    m["tire.DAMP_YAW"] = &p->tire_parameters->DAMP_YAW;
    m["tire.DAMP_WINDUP"] = &p->tire_parameters->DAMP_WINDUP;
    m["tire.DAMP_RESIDUAL"] = &p->tire_parameters->DAMP_RESIDUAL;
    m["tire.DAMP_VLOW"] = &p->tire_parameters->DAMP_VLOW;
    m["tire.Q_BVX"] = &p->tire_parameters->Q_BVX;
    m["tire.Q_BVT"] = &p->tire_parameters->Q_BVT;
    m["tire.PCFX1"] = &p->tire_parameters->PCFX1;
    m["tire.PCFX2"] = &p->tire_parameters->PCFX2;
    m["tire.PCFX3"] = &p->tire_parameters->PCFX3;
    m["tire.PCFY1"] = &p->tire_parameters->PCFY1;
    m["tire.PCFY2"] = &p->tire_parameters->PCFY2;
    m["tire.PCFY3"] = &p->tire_parameters->PCFY3;
    m["tire.PCMZ1"] = &p->tire_parameters->PCMZ1;
    m["tire.Q_RA1"] = &p->tire_parameters->Q_RA1;
    m["tire.Q_RA2"] = &p->tire_parameters->Q_RA2;
    m["tire.Q_RB1"] = &p->tire_parameters->Q_RB1;
    m["tire.Q_RB2"] = &p->tire_parameters->Q_RB2;
    m["tire.ELLIPS_SHIFT"] = &p->tire_parameters->ELLIPS_SHIFT;
    m["tire.ELLIPS_LENGTH"] = &p->tire_parameters->ELLIPS_LENGTH;
    m["tire.ELLIPS_HEIGHT"] = &p->tire_parameters->ELLIPS_HEIGHT;
    m["tire.ELLIPS_ORDER"] = &p->tire_parameters->ELLIPS_ORDER;
    m["tire.ELLIPS_MAX_STEP"] = &p->tire_parameters->ELLIPS_MAX_STEP;
    m["tire.ELLIPS_NWIDTH"] = &p->tire_parameters->ELLIPS_NWIDTH;
    m["tire.ELLIPS_NLENGTH"] = &p->tire_parameters->ELLIPS_NLENGTH;
    m["tire.PRESMIN"] = &p->tire_parameters->PRESMIN;
    m["tire.PRESMAX"] = &p->tire_parameters->PRESMAX;
    m["tire.FZMIN"] = &p->tire_parameters->FZMIN;
    m["tire.FZMAX"] = &p->tire_parameters->FZMAX;
    m["tire.KPUMIN"] = &p->tire_parameters->KPUMIN;
    m["tire.KPUMAX"] = &p->tire_parameters->KPUMAX;
    m["tire.ALPMIN"] = &p->tire_parameters->ALPMIN;
    m["tire.ALPMAX"] = &p->tire_parameters->ALPMAX;
    m["tire.CAMMIN"] = &p->tire_parameters->CAMMIN;
    m["tire.CAMMAX"] = &p->tire_parameters->CAMMAX;
    m["tire.LFZO"] = &p->tire_parameters->LFZO;
    m["tire.LCX"] = &p->tire_parameters->LCX;
    m["tire.LMUX"] = &p->tire_parameters->LMUX;
    m["tire.LEX"] = &p->tire_parameters->LEX;
    m["tire.LKX"] = &p->tire_parameters->LKX;
    m["tire.LHX"] = &p->tire_parameters->LHX;
    m["tire.LVX"] = &p->tire_parameters->LVX;
    m["tire.LCY"] = &p->tire_parameters->LCY;
    m["tire.LMUY"] = &p->tire_parameters->LMUY;
    m["tire.LEY"] = &p->tire_parameters->LEY;
    m["tire.LKY"] = &p->tire_parameters->LKY;
    m["tire.LHY"] = &p->tire_parameters->LHY;
    m["tire.LVY"] = &p->tire_parameters->LVY;
    m["tire.LTR"] = &p->tire_parameters->LTR;
    m["tire.LRES"] = &p->tire_parameters->LRES;
    m["tire.LXAL"] = &p->tire_parameters->LXAL;
    m["tire.LYKA"] = &p->tire_parameters->LYKA;
    m["tire.LVYKA"] = &p->tire_parameters->LVYKA;
    m["tire.LS"] = &p->tire_parameters->LS;
    m["tire.LKYC"] = &p->tire_parameters->LKYC;
    m["tire.LKZC"] = &p->tire_parameters->LKZC;
    m["tire.LVMX"] = &p->tire_parameters->LVMX;
    m["tire.LMX"] = &p->tire_parameters->LMX;
    m["tire.LMY"] = &p->tire_parameters->LMY;
    m["tire.LMP"] = &p->tire_parameters->LMP;
    m["tire.PCX1"] = &p->tire_parameters->PCX1;
    m["tire.PDX1"] = &p->tire_parameters->PDX1;
    m["tire.PDX2"] = &p->tire_parameters->PDX2;
    m["tire.PDX3"] = &p->tire_parameters->PDX3;
    m["tire.PEX1"] = &p->tire_parameters->PEX1;
    m["tire.PEX2"] = &p->tire_parameters->PEX2;
    m["tire.PEX3"] = &p->tire_parameters->PEX3;
    m["tire.PEX4"] = &p->tire_parameters->PEX4;
    m["tire.PKX1"] = &p->tire_parameters->PKX1;
    m["tire.PKX2"] = &p->tire_parameters->PKX2;
    m["tire.PKX3"] = &p->tire_parameters->PKX3;
    m["tire.PHX1"] = &p->tire_parameters->PHX1;
    m["tire.PHX2"] = &p->tire_parameters->PHX2;
    m["tire.PVX1"] = &p->tire_parameters->PVX1;
    m["tire.PVX2"] = &p->tire_parameters->PVX2;
    m["tire.PPX1"] = &p->tire_parameters->PPX1;
    m["tire.PPX2"] = &p->tire_parameters->PPX2;
    m["tire.PPX3"] = &p->tire_parameters->PPX3;
    m["tire.PPX4"] = &p->tire_parameters->PPX4;
    m["tire.RBX1"] = &p->tire_parameters->RBX1;
    m["tire.RBX2"] = &p->tire_parameters->RBX2;
    m["tire.RBX3"] = &p->tire_parameters->RBX3;
    m["tire.RCX1"] = &p->tire_parameters->RCX1;
    m["tire.REX1"] = &p->tire_parameters->REX1;
    m["tire.REX2"] = &p->tire_parameters->REX2;
    m["tire.RHX1"] = &p->tire_parameters->RHX1;
    m["tire.QSX1"] = &p->tire_parameters->QSX1;
    m["tire.QSX2"] = &p->tire_parameters->QSX2;
    m["tire.QSX3"] = &p->tire_parameters->QSX3;
    m["tire.QSX4"] = &p->tire_parameters->QSX4;
    m["tire.QSX5"] = &p->tire_parameters->QSX5;
    m["tire.QSX6"] = &p->tire_parameters->QSX6;
    m["tire.QSX7"] = &p->tire_parameters->QSX7;
    m["tire.QSX8"] = &p->tire_parameters->QSX8;
    m["tire.QSX9"] = &p->tire_parameters->QSX9;
    m["tire.QSX10"] = &p->tire_parameters->QSX10;
    m["tire.QSX11"] = &p->tire_parameters->QSX11;
    m["tire.QSX12"] = &p->tire_parameters->QSX12;
    m["tire.QSX13"] = &p->tire_parameters->QSX13;
    m["tire.QSX14"] = &p->tire_parameters->QSX14;
    m["tire.PPMX1"] = &p->tire_parameters->PPMX1;
    m["tire.PCY1"] = &p->tire_parameters->PCY1;
    m["tire.PDY1"] = &p->tire_parameters->PDY1;
    m["tire.PDY2"] = &p->tire_parameters->PDY2;
    m["tire.PDY3"] = &p->tire_parameters->PDY3;
    m["tire.PEY1"] = &p->tire_parameters->PEY1;
    m["tire.PEY2"] = &p->tire_parameters->PEY2;
    m["tire.PEY3"] = &p->tire_parameters->PEY3;
    m["tire.PEY4"] = &p->tire_parameters->PEY4;
    m["tire.PEY5"] = &p->tire_parameters->PEY5;
    m["tire.PKY1"] = &p->tire_parameters->PKY1;
    m["tire.PKY2"] = &p->tire_parameters->PKY2;
    m["tire.PKY3"] = &p->tire_parameters->PKY3;
    m["tire.PKY4"] = &p->tire_parameters->PKY4;
    m["tire.PKY5"] = &p->tire_parameters->PKY5;
    m["tire.PKY6"] = &p->tire_parameters->PKY6;
    m["tire.PKY7"] = &p->tire_parameters->PKY7;
    m["tire.PHY1"] = &p->tire_parameters->PHY1;
    m["tire.PHY2"] = &p->tire_parameters->PHY2;
    m["tire.PVY1"] = &p->tire_parameters->PVY1;
    m["tire.PVY2"] = &p->tire_parameters->PVY2;
    m["tire.PVY3"] = &p->tire_parameters->PVY3;
    m["tire.PVY4"] = &p->tire_parameters->PVY4;
    m["tire.PPY1"] = &p->tire_parameters->PPY1;
    m["tire.PPY2"] = &p->tire_parameters->PPY2;
    m["tire.PPY3"] = &p->tire_parameters->PPY3;
    m["tire.PPY4"] = &p->tire_parameters->PPY4;
    m["tire.PPY5"] = &p->tire_parameters->PPY5;
    m["tire.RBY1"] = &p->tire_parameters->RBY1;
    m["tire.RBY2"] = &p->tire_parameters->RBY2;
    m["tire.RBY3"] = &p->tire_parameters->RBY3;
    m["tire.RBY4"] = &p->tire_parameters->RBY4;
    m["tire.RCY1"] = &p->tire_parameters->RCY1;
    m["tire.REY1"] = &p->tire_parameters->REY1;
    m["tire.REY2"] = &p->tire_parameters->REY2;
    m["tire.RHY1"] = &p->tire_parameters->RHY1;
    m["tire.RHY2"] = &p->tire_parameters->RHY2;
    m["tire.RVY1"] = &p->tire_parameters->RVY1;
    m["tire.RVY2"] = &p->tire_parameters->RVY2;
    m["tire.RVY3"] = &p->tire_parameters->RVY3;
    m["tire.RVY4"] = &p->tire_parameters->RVY4;
    m["tire.RVY5"] = &p->tire_parameters->RVY5;
    m["tire.RVY6"] = &p->tire_parameters->RVY6;
    m["tire.QSY1"] = &p->tire_parameters->QSY1;
    m["tire.QSY2"] = &p->tire_parameters->QSY2;
    m["tire.QSY3"] = &p->tire_parameters->QSY3;
    m["tire.QSY4"] = &p->tire_parameters->QSY4;
    m["tire.QSY5"] = &p->tire_parameters->QSY5;
    m["tire.QSY6"] = &p->tire_parameters->QSY6;
    m["tire.QSY7"] = &p->tire_parameters->QSY7;
    m["tire.QSY8"] = &p->tire_parameters->QSY8;
    m["tire.QBZ1"] = &p->tire_parameters->QBZ1;
    m["tire.QBZ2"] = &p->tire_parameters->QBZ2;
    m["tire.QBZ3"] = &p->tire_parameters->QBZ3;
    m["tire.QBZ4"] = &p->tire_parameters->QBZ4;
    m["tire.QBZ5"] = &p->tire_parameters->QBZ5;
    m["tire.QBZ9"] = &p->tire_parameters->QBZ9;
    m["tire.QBZ10"] = &p->tire_parameters->QBZ10;
    m["tire.QCZ1"] = &p->tire_parameters->QCZ1;
    m["tire.QDZ1"] = &p->tire_parameters->QDZ1;
    m["tire.QDZ2"] = &p->tire_parameters->QDZ2;
    m["tire.QDZ3"] = &p->tire_parameters->QDZ3;
    m["tire.QDZ4"] = &p->tire_parameters->QDZ4;
    m["tire.QDZ6"] = &p->tire_parameters->QDZ6;
    m["tire.QDZ7"] = &p->tire_parameters->QDZ7;
    m["tire.QDZ8"] = &p->tire_parameters->QDZ8;
    m["tire.QDZ9"] = &p->tire_parameters->QDZ9;
    m["tire.QDZ10"] = &p->tire_parameters->QDZ10;
    m["tire.QDZ11"] = &p->tire_parameters->QDZ11;
    m["tire.QEZ1"] = &p->tire_parameters->QEZ1;
    m["tire.QEZ2"] = &p->tire_parameters->QEZ2;
    m["tire.QEZ3"] = &p->tire_parameters->QEZ3;
    m["tire.QEZ4"] = &p->tire_parameters->QEZ4;
    m["tire.QEZ5"] = &p->tire_parameters->QEZ5;
    m["tire.QHZ1"] = &p->tire_parameters->QHZ1;
    m["tire.QHZ2"] = &p->tire_parameters->QHZ2;
    m["tire.QHZ3"] = &p->tire_parameters->QHZ3;
    m["tire.QHZ4"] = &p->tire_parameters->QHZ4;
    m["tire.PPZ1"] = &p->tire_parameters->PPZ1;
    m["tire.PPZ2"] = &p->tire_parameters->PPZ2;
    m["tire.SSZ1"] = &p->tire_parameters->SSZ1;
    m["tire.SSZ2"] = &p->tire_parameters->SSZ2;
    m["tire.SSZ3"] = &p->tire_parameters->SSZ3;
    m["tire.SSZ4"] = &p->tire_parameters->SSZ4;
    m["tire.PECP1"] = &p->tire_parameters->PECP1;
    m["tire.PECP2"] = &p->tire_parameters->PECP2;
    m["tire.PDXP1"] = &p->tire_parameters->PDXP1;
    m["tire.PDXP2"] = &p->tire_parameters->PDXP2;
    m["tire.PDXP3"] = &p->tire_parameters->PDXP3;
    m["tire.PDXP4"] = &p->tire_parameters->PDXP4;
    m["tire.PDYP1"] = &p->tire_parameters->PDYP1;
    m["tire.PDYP2"] = &p->tire_parameters->PDYP2;
    m["tire.PDYP3"] = &p->tire_parameters->PDYP3;
    m["tire.PDYP4"] = &p->tire_parameters->PDYP4;
    m["tire.PKYP1"] = &p->tire_parameters->PKYP1;
    m["tire.PHYP1"] = &p->tire_parameters->PHYP1;
    m["tire.PHYP2"] = &p->tire_parameters->PHYP2;
    m["tire.PHYP3"] = &p->tire_parameters->PHYP3;
    m["tire.PHYP4"] = &p->tire_parameters->PHYP4;
    m["tire.QDTP1"] = &p->tire_parameters->QDTP1;
    m["tire.QBRP1"] = &p->tire_parameters->QBRP1;
    m["tire.QCRP1"] = &p->tire_parameters->QCRP1;
    m["tire.QCRP2"] = &p->tire_parameters->QCRP2;
    m["tire.QDRP1"] = &p->tire_parameters->QDRP1;
    m["tire.QDRP2"] = &p->tire_parameters->QDRP2;
    m["tire.PA1"] = &p->tire_parameters->PA1;
    m["tire.PA2"] = &p->tire_parameters->PA2;
    m["tire.PB1"] = &p->tire_parameters->PB1;
    m["tire.PB2"] = &p->tire_parameters->PB2;
    m["tire.PB3"] = &p->tire_parameters->PB3;
    m["tire.PAE"] = &p->tire_parameters->PAE;
    m["tire.PBE"] = &p->tire_parameters->PBE;
    m["tire.PCE"] = &p->tire_parameters->PCE;
    m["tire.PLS"] = &p->tire_parameters->PLS;
    m["tire.PW1"] = &p->tire_parameters->PW1;
    m["tire.PW2"] = &p->tire_parameters->PW2;
    m["tire.PW3"] = &p->tire_parameters->PW3;
    m["tire.PW4"] = &p->tire_parameters->PW4;
    m["tire.N_WIDTH"] = &p->tire_parameters->N_WIDTH;
    m["tire.N_WIDTH_HP"] = &p->tire_parameters->N_WIDTH_HP;
    m["tire.N_LENGTH"] = &p->tire_parameters->N_LENGTH;
    m["tire.N_LENGTH_HP"] = &p->tire_parameters->N_LENGTH_HP;
    m["tire.ROAD_SPACING"] = &p->tire_parameters->ROAD_SPACING;
    m["tire.ROAD_SPACING_HP"] = &p->tire_parameters->ROAD_SPACING_HP;
    m["tire.MAX_HEIGHT"] = &p->tire_parameters->MAX_HEIGHT;
    m["tire.CONTACT_THREADS"] = &p->tire_parameters->CONTACT_THREADS;
    m["tire.CONTACT_THREADS_HP"] = &p->tire_parameters->CONTACT_THREADS_HP;
    m["tire.EPSNL"] = &p->tire_parameters->EPSNL;
    m["tire.MC"] = &p->tire_parameters->MC;
    m["tire.IC"] = &p->tire_parameters->IC;
    m["tire.KX"] = &p->tire_parameters->KX;
    m["tire.KY"] = &p->tire_parameters->KY;
    m["tire.KP"] = &p->tire_parameters->KP;
    m["tire.CX"] = &p->tire_parameters->CX;
    m["tire.CY"] = &p->tire_parameters->CY;
    m["tire.CP"] = &p->tire_parameters->CP;
    m["tire.EP"] = &p->tire_parameters->EP;
    m["tire.EP12"] = &p->tire_parameters->EP12;
    m["tire.BF2"] = &p->tire_parameters->BF2;
    m["tire.BP1"] = &p->tire_parameters->BP1;
    m["tire.BP2"] = &p->tire_parameters->BP2;
    m["tire.BP3"] = &p->tire_parameters->BP3;
    m["tire.BP4"] = &p->tire_parameters->BP4;
    m["tire.CXZ1"] = &p->tire_parameters->CXZ1;
    m["tire.CXZ2"] = &p->tire_parameters->CXZ2;
    m["tire.CXP1"] = &p->tire_parameters->CXP1;
    m["tire.CXX1"] = &p->tire_parameters->CXX1;
    m["tire.CYZ1"] = &p->tire_parameters->CYZ1;
    m["tire.CYZ2"] = &p->tire_parameters->CYZ2;
    m["tire.CYP1"] = &p->tire_parameters->CYP1;
    m["tire.CYY1"] = &p->tire_parameters->CYY1;
    m["tire.CPZ1"] = &p->tire_parameters->CPZ1;
    m["tire.TYRE_MASS"] = &p->tire_parameters->TYRE_MASS;
    m["tire.QMB"] = &p->tire_parameters->QMB;
    m["tire.QMC"] = &p->tire_parameters->QMC;
    m["tire.QIBY"] = &p->tire_parameters->QIBY;
    m["tire.QIBXZ"] = &p->tire_parameters->QIBXZ;
    m["tire.QIC"] = &p->tire_parameters->QIC;
    m["tire.QCBXZ"] = &p->tire_parameters->QCBXZ;
    m["tire.QCBY"] = &p->tire_parameters->QCBY;
    m["tire.QCBTH"] = &p->tire_parameters->QCBTH;
    m["tire.QCBGM"] = &p->tire_parameters->QCBGM;
    m["tire.QKBXZ"] = &p->tire_parameters->QKBXZ;
    m["tire.QKBY"] = &p->tire_parameters->QKBY;
    m["tire.QKBTH"] = &p->tire_parameters->QKBTH;
    m["tire.QKBGM"] = &p->tire_parameters->QKBGM;
    m["tire.QCCX"] = &p->tire_parameters->QCCX;
    m["tire.QCCY"] = &p->tire_parameters->QCCY;
    m["tire.QCCFI"] = &p->tire_parameters->QCCFI;
    m["tire.QKBX"] = &p->tire_parameters->QKBX;
    m["tire.QKCY"] = &p->tire_parameters->QKCY;
    m["tire.QKCFI"] = &p->tire_parameters->QKCFI;
    m["tire.QBVXZ"] = &p->tire_parameters->QBVXZ;
    m["tire.QBVTH"] = &p->tire_parameters->QBVTH;
    m["tire.QRE0"] = &p->tire_parameters->QRE0;
    m["tire.QV1"] = &p->tire_parameters->QV1;
    m["tire.QV2"] = &p->tire_parameters->QV2;
    m["tire.QFCX1"] = &p->tire_parameters->QFCX1;
    m["tire.QFCY1"] = &p->tire_parameters->QFCY1;
    m["tire.QFCG1"] = &p->tire_parameters->QFCG1;
    m["tire.QFZ1"] = &p->tire_parameters->QFZ1;
    m["tire.QFZ2"] = &p->tire_parameters->QFZ2;
    m["tire.QFZ3"] = &p->tire_parameters->QFZ3;
    m["tire.QPFZ1"] = &p->tire_parameters->QPFZ1;
    m["tire.Amu"] = &p->tire_parameters->Amu;

    // Aliases for tuning configs
    m["tire.longitudinal_peak_pdx1"] = &p->tire_parameters->PDX1;
    m["tire.longitudinal_stiffness_pkx1"] = &p->tire_parameters->PKX1;
    m["tire.longitudinal_curvature_pex1"] = &p->tire_parameters->PEX1;
    m["tire.lateral_peak_pdy1"] = &p->tire_parameters->PDY1;
    m["tire.lateral_stiffness_pky1"] = &p->tire_parameters->PKY1;
    m["tire.lateral_curvature_pey1"] = &p->tire_parameters->PEY1;
    m["tire.structural_longitudinal_stiffness"] = &p->tire_parameters->LONGITUDINAL_STIFFNESS;
    m["tire.structural_lateral_stiffness"] = &p->tire_parameters->LATERAL_STIFFNESS;
    m["tire.structural_yaw_stiffness"] = &p->tire_parameters->YAW_STIFFNESS;
    m["tire.longitudinal_peak_scale"] = &p->tire_parameters->LMUX;
    m["tire.longitudinal_stiffness_scale"] = &p->tire_parameters->LKX;
    m["tire.lateral_shape_scale"] = &p->tire_parameters->LCY;
    m["tire.lateral_peak_scale"] = &p->tire_parameters->LMUY;
    m["tire.lateral_curvature_scale"] = &p->tire_parameters->LEY;
    m["tire.lateral_stiffness_scale"] = &p->tire_parameters->LKY;
    m["tire.combined_slip_lateral_stiffness_scale"] = &p->tire_parameters->LYKA;
  }
  if (p->transmission_parameters) {
    m["transmission.gear_ratio"] = &p->transmission_parameters->gear_ratio;
    m["transmission.efficiency"] = &p->transmission_parameters->efficiency;
    m["transmission.kv"] = &p->transmission_parameters->kv;
    m["transmission.t_max"] = &p->transmission_parameters->t_max;
    m["transmission.viscous_drag_coeff"] = &p->transmission_parameters->viscous_drag_coeff;
    m["transmission.coulomb_drag"] = &p->transmission_parameters->coulomb_drag;
    m["transmission.coulomb_smooth_stiffness"] = &p->transmission_parameters->coulomb_smooth_stiffness;
  }

  return m;
}

void apply_parameter_override(
    std::shared_ptr<common_lib::car_parameters::CarParameters> car_params,
    const std::string& name, double val) {
  auto ptrs = get_parameter_ptrs(car_params);
  auto it = ptrs.find(name);
  if (it != ptrs.end()) {
    *(it->second) = val;
  } else {
    std::cerr << "Warning: Unknown parameter override '" << name << "'" << std::endl;
  }
}

double get_baseline_value(
    std::shared_ptr<common_lib::car_parameters::CarParameters> car_params,
    const std::string& name) {
  auto ptrs = get_parameter_ptrs(car_params);
  auto it = ptrs.find(name);
  if (it != ptrs.end()) {
    return *(it->second);
  }
  return 0.0;
}

// Polymorphic base for standalone simulator models
class StandaloneVehicleModel {
public:
  virtual ~StandaloneVehicleModel() = default;
  virtual void step(double dt, const common_lib::structures::Wheels& throttle, double steering_angle, SimState& state) = 0;
};

// Standalone FSFEUP02 implementation
class FSFEUP02StandaloneModel : public StandaloneVehicleModel {
private:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_params;
  std::shared_ptr<TireModel> tire_model;
  std::shared_ptr<MotorModel> motor;
  std::shared_ptr<BatteryModel> battery;
  std::shared_ptr<TransmissionModel> transmission;
  std::shared_ptr<InverterModel> inverter;
  std::shared_ptr<BrakeModel> brake;
  std::shared_ptr<AeroModel> aero;
  std::shared_ptr<LoadTransferModel> load_transfer;
  std::shared_ptr<SteeringModel> steering;
  std::shared_ptr<SteeringMotorModel> steering_motor;

  double calculate_powertrain_torque(double throttle_input, double dt, SimState& state) {
    common_lib::structures::Wheels w_speeds(state.wl_fl, state.wl_fr, state.wl_rl, state.wl_rr);
    double motor_omega = transmission->calculate_motor_omega(w_speeds);
    double motor_rpm = std::abs(motor_omega * 60.0f / (2.0f * M_PI));

    double max_motor_torque = motor->get_max_torque_at_rpm(motor_rpm);
    double reference_motor_torque = throttle_input * max_motor_torque;

    const double min_omega_for_power = 10.0;
    double omega_sign_source =
        std::abs(motor_omega) > 1e-3 ? motor_omega : reference_motor_torque;

    double omega_for_power =
        std::copysign(std::max(std::abs(motor_omega), min_omega_for_power),
                      omega_sign_source);

    double mechanical_power_request =
        reference_motor_torque * omega_for_power;

    double motor_efficiency = motor->get_efficiency(std::abs(reference_motor_torque), motor_rpm);
    double inverter_efficiency = inverter->get_efficiency();
    double total_efficiency = motor_efficiency * inverter_efficiency;

    double battery_power_request;
    if (mechanical_power_request >= 0.0) {
      battery_power_request = mechanical_power_request / total_efficiency;
    } else {
      battery_power_request = mechanical_power_request * total_efficiency;
    }

    double battery_voltage = battery->get_voltage();
    double requested_battery_current =
        battery_power_request / battery_voltage;
    double allowed_battery_current =
        battery->calculate_allowed_current(requested_battery_current);

    double allowed_battery_voltage =
        battery->get_voltage(allowed_battery_current);
    double allowed_battery_power =
        allowed_battery_current * allowed_battery_voltage;

    double allowed_mechanical_power;
    if (allowed_battery_power >= 0.0) {
      allowed_mechanical_power = allowed_battery_power * total_efficiency;
    } else {
      allowed_mechanical_power = allowed_battery_power / total_efficiency;
    }

    double actual_motor_torque = std::copysign(
        std::min(std::abs(allowed_mechanical_power / omega_for_power),
                 std::abs(reference_motor_torque)),
        reference_motor_torque);

    double motor_phase_current =
        std::abs(actual_motor_torque) / car_params->motor_parameters->kt_constant;

    double max_inverter_phase_current =
        car_params->inverter_parameters->max_phase_current;
    if (motor_phase_current > max_inverter_phase_current) {
      double limited_torque =
          max_inverter_phase_current *
          car_params->motor_parameters->kt_constant;

      actual_motor_torque =
          std::copysign(limited_torque, actual_motor_torque);

      motor_phase_current = max_inverter_phase_current;
    }

    const double signed_motor_phase_current =
        std::abs(allowed_battery_current) > 1e-9
            ? std::copysign(motor_phase_current, allowed_battery_current)
            : 0.0;

    battery->update_state(allowed_battery_current, dt);
    motor->update_state(signed_motor_phase_current,
                         actual_motor_torque,
                         dt);

    state.motor_torque = actual_motor_torque;
    state.motor_omega = motor_omega;
    state.motor_current = motor->get_current();
    state.battery_current = battery->get_current();
    state.battery_voltage = battery->get_voltage();

    return actual_motor_torque;
  }

  StateVec get_state_derivative(
      const StateVec& s,
      double motor_torque,
      const common_lib::structures::Wheels& brake_torques,
      double steering_target,
      double dt,
      const Eigen::Vector4d& last_slip_ratio,
      const Eigen::Vector4d& last_slip_angle,
      Eigen::Vector4d& out_slip_ratio,
      Eigen::Vector4d& out_slip_angle
  ) {
    StateVec ds = StateVec::Zero();
    const common_lib::structures::Wheels wheel_speeds(s(7), s(8), s(9), s(10));

    const common_lib::structures::Wheels wheel_torques =
        transmission->calculate_wheel_torques(motor_torque, wheel_speeds);
    const Eigen::Vector4d torques(wheel_torques.front_left, wheel_torques.front_right,
                                  wheel_torques.rear_left, wheel_torques.rear_right);

    Eigen::Vector4d wheel_angles = steering->calculate_steering_angles(s(6));

    const Eigen::Vector3d aero_forces =
        aero->aero_forces(Eigen::Vector3d(s(0), s(1), s(2)));

    const common_lib::structures::Wheels load_distribution = load_transfer->compute_loads(
        LoadTransferInput{s(11), s(12), aero_forces[2]});
    const Eigen::Vector4d vertical_loads(load_distribution.front_left, load_distribution.front_right,
                                         load_distribution.rear_left, load_distribution.rear_right);

    Eigen::VectorXd tire_forces(16);
    TireInput tire_input;
    tire_input.dt = dt;
    tire_input.vx = s(0);
    tire_input.vy = s(1);
    tire_input.yaw_rate = s(2);
    tire_input.last_slip_ratio = last_slip_ratio;
    tire_input.last_slip_angle = last_slip_angle;

    for (Tire tire : {FL, FR, RL, RR}) {
      tire_input.tire = tire;
      tire_input.steering_angle = wheel_angles(tire);
      tire_input.wheel_angular_speed = s(7 + tire);
      tire_input.vertical_load = vertical_loads(tire);
      tire_forces.segment<4>(tire * 4) = tire_model->calculate_tire_forces(tire_input);
      out_slip_ratio(tire) = tire_input.slip_ratio;
      out_slip_angle(tire) = tire_input.slip_angle;
    }

    double total_fx = aero_forces[0];
    double total_fy = aero_forces[1];
    double total_torque = 0.0;

    const double lr = car_params->cg_2_rear_axis;
    const double lf = car_params->wheelbase - lr;
    const double half_width = car_params->track_width / 2.0;
    const double wheel_radius = car_params->tire_parameters->effective_tire_r;
    const Eigen::Vector4d brake_torques_by_tire(brake_torques.front_left,
                                                brake_torques.front_right,
                                                brake_torques.rear_left,
                                                brake_torques.rear_right);

    for (Tire tire : {FL, FR, RL, RR}) {
      double fx_tire = tire_forces(tire * 4);
      const double fy_tire = tire_forces(tire * 4 + 1);
      const double mz_tire = tire_forces(tire * 4 + 3);
      const double cos_delta = std::cos(wheel_angles(tire));
      const double sin_delta = std::sin(wheel_angles(tire));
      const double brake_torque = brake_torques_by_tire(tire);

      double vcx_tire = s(0);
      if (tire == FL || tire == FR) {
        vcx_tire = s(0) * cos_delta + (s(1) + lf * s(2)) * sin_delta;
      } else {
        vcx_tire = s(0) - (tire == RL ? half_width : -half_width) * s(2);
      }

      if (brake_torque > 0.0) {
        const double brake_sign =
            2.0 / M_PI * std::atan(10.0 * vcx_tire);
        fx_tire -= brake_torque * brake_sign / std::max(wheel_radius, kEpsilon);
      }

      const double fx_vehicle = fx_tire * cos_delta - fy_tire * sin_delta;
      const double fy_vehicle = fx_tire * sin_delta + fy_tire * cos_delta;
      const double arm_x = (tire == FL || tire == FR) ? lf : -lr;
      const double arm_y = (tire == FL || tire == RL) ? half_width : -half_width;

      total_fx += fx_vehicle;
      total_fy += fy_vehicle;
      total_torque += arm_x * fy_vehicle - arm_y * fx_vehicle + mz_tire;
    }

    const double ax = total_fx / car_params->total_mass + s(1) * s(2);
    const double ay = total_fy / car_params->total_mass - s(0) * s(2);
    ds(0) = ax;
    ds(1) = ay;
    ds(11) = ax - s(11);
    ds(12) = ay - s(12);
    ds(2) = total_torque / car_params->Izz;
    ds(3) = s(2);
    ds(4) = s(0) * std::cos(s(3)) - s(1) * std::sin(s(3));
    ds(5) = s(0) * std::sin(s(3)) + s(1) * std::cos(s(3));
    ds(6) = steering_motor->compute_steering_rate(s(6), steering_target);

    const double inertia = car_params->tire_parameters->wheel_inertia;
    for (Tire tire : {FL, FR, RL, RR}) {
      const double wheel_omega = s(7 + tire);
      double net_torque =
          torques(tire) - tire_forces(tire * 4) * wheel_radius - tire_forces(tire * 4 + 2);
      const double brake_sign = 2.0 / M_PI * std::atan(10.0 * wheel_omega);
      const double brake_torque = brake_torques_by_tire(tire);
      net_torque -= brake_torque * brake_sign;

      if (tire == FL || tire == FR) {
        net_torque -= car_params->front_bearing_drag * wheel_omega;
      }

      const double wheel_acceleration = net_torque / inertia;
      if (brake_torque > 0.0 && std::abs(wheel_omega) < 0.5 &&
          wheel_acceleration * wheel_omega <= 0.0) {
        ds(7 + tire) = -wheel_omega / std::max(dt, kEpsilon);
      } else {
        ds(7 + tire) = wheel_acceleration;
      }
    }

    return ds;
  }

public:
  FSFEUP02StandaloneModel(const InvictaSimParameters& p) {
    car_params = p.car_parameters;
    tire_model = tire_models_map.at(p.tire_model)(car_params);
    motor = motor_models_map.at(p.motor_model)(car_params);
    battery = battery_models_map.at(p.battery_model)(car_params);
    transmission = transmission_models_map.at(p.transmission_model)(car_params);
    inverter = inverter_models_map.at(p.inverter_model)(car_params);
    brake = brake_models_map.at(p.brake_model)(car_params);
    aero = aero_models_map.at(p.aero_model)(car_params);
    load_transfer = load_transfer_models_map.at(p.load_transfer_model)(car_params);
    steering = steering_models_map.at(p.steering_model)(car_params);
    steering_motor = steering_motor_models_map.at(p.steering_motor_model)(car_params);
  }

  void step(double dt, const common_lib::structures::Wheels& throttle, double steering_angle, SimState& state) override {
    const double throttle_input = (throttle.rear_left + throttle.rear_right) / 2.0;
    common_lib::structures::Wheels brake_torques;
    double inverter_command = throttle_input;

    const double motor_input = inverter->calculate_inverter_throttle(inverter_command, dt);
    const double motor_torque = calculate_powertrain_torque(motor_input, dt, state);

    StateVec s;
    s << state.vx, state.vy, state.yaw_rate, state.yaw, state.x, state.y,
         state.steering_angle, state.wl_fl, state.wl_fr, state.wl_rl, state.wl_rr,
         state.ax, state.ay;

    Eigen::Vector4d k1_ratio, k1_angle;
    const StateVec k1 = get_state_derivative(
        s, motor_torque, brake_torques, steering_angle, dt,
        state.last_slip_ratio, state.last_slip_angle, k1_ratio, k1_angle
    );

    Eigen::Vector4d k2_ratio, k2_angle;
    const StateVec k2 = get_state_derivative(
        s + 0.5 * dt * k1, motor_torque, brake_torques, steering_angle, dt,
        state.last_slip_ratio, state.last_slip_angle, k2_ratio, k2_angle
    );

    Eigen::Vector4d k3_ratio, k3_angle;
    const StateVec k3 = get_state_derivative(
        s + 0.5 * dt * k2, motor_torque, brake_torques, steering_angle, dt,
        state.last_slip_ratio, state.last_slip_angle, k3_ratio, k3_angle
    );

    Eigen::Vector4d k4_ratio, k4_angle;
    const StateVec k4 = get_state_derivative(
        s + dt * k3, motor_torque, brake_torques, steering_angle, dt,
        state.last_slip_ratio, state.last_slip_angle, k4_ratio, k4_angle
    );

    StateVec s_next = s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    state.vx = s_next(0);
    state.vy = s_next(1);
    state.yaw_rate = s_next(2);
    state.yaw = s_next(3);
    state.x = s_next(4);
    state.y = s_next(5);
    state.steering_angle = s_next(6);
    state.wl_fl = s_next(7);
    state.wl_fr = s_next(8);
    state.wl_rl = s_next(9);
    state.wl_rr = s_next(10);
    state.ax = s_next(11);
    state.ay = s_next(12);

    double speed = std::sqrt(state.vx * state.vx + state.vy * state.vy);
    if (speed < 0.05 && std::abs(throttle_input) < 0.01) {
      state.vx = 0.0;
      state.vy = 0.0;
      state.ax = 0.0;
      state.ay = 0.0;
      state.yaw_rate = 0.0;
      state.wl_fl = 0.0;
      state.wl_fr = 0.0;
      state.wl_rl = 0.0;
      state.wl_rr = 0.0;
      state.last_slip_ratio.setZero();
      state.last_slip_angle.setZero();
    } else {
      state.last_slip_ratio = k1_ratio;
      state.last_slip_angle = k1_angle;
    }

    if (state.yaw > M_PI) state.yaw -= 2.0 * M_PI;
    if (state.yaw < -M_PI) state.yaw += 2.0 * M_PI;
  }
};

// Standalone FSFEUP03 implementation
class FSFEUP03StandaloneModel : public StandaloneVehicleModel {
private:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_params;
  std::shared_ptr<TireModel> tire_model;
  std::shared_ptr<MotorModel> motor;
  std::shared_ptr<BatteryModel> battery;
  std::shared_ptr<TransmissionModel> transmission;
  std::shared_ptr<InverterModel> inverter;
  std::shared_ptr<BrakeModel> brake;
  std::shared_ptr<AeroModel> aero;
  std::shared_ptr<LoadTransferModel> load_transfer;
  std::shared_ptr<SteeringModel> steering;
  std::shared_ptr<SteeringMotorModel> steering_motor;

  double calculate_powertrain_torque(double throttle_input, double dt, SimState& state) {
    double motor_omega = transmission->calculate_motor_omega(
        common_lib::structures::Wheels(state.wl_fl, state.wl_fr, state.wl_rl, state.wl_rr)
    );
    double motor_rpm = std::abs(motor_omega * 60.0f / (2.0f * M_PI));

    double max_motor_torque = motor->get_max_torque_at_rpm(motor_rpm);
    double reference_motor_torque = throttle_input * max_motor_torque;

    double motor_efficiency = motor->get_efficiency(std::abs(reference_motor_torque), motor_rpm);

    const double current_magnitude =
        std::abs(reference_motor_torque) /
        (car_params->motor_parameters->kt_constant * std::max(motor_efficiency, 0.05));
    const double mechanical_power = reference_motor_torque * motor_omega;
    const double current_sign =
        std::abs(motor_omega) > 1e-3 ? std::copysign(1.0, mechanical_power)
                                     : std::copysign(1.0, reference_motor_torque);
    double requested_motor_current = current_sign * current_magnitude;

    double allowed_motor_current = battery->calculate_allowed_current(requested_motor_current);

    double actual_motor_torque =
        allowed_motor_current * car_params->motor_parameters->kt_constant * motor_efficiency;

    battery->update_state(allowed_motor_current, dt);
    motor->update_state(allowed_motor_current, actual_motor_torque, dt);

    state.motor_torque = actual_motor_torque;
    state.motor_omega = motor_omega;
    state.motor_current = motor->get_current();
    state.battery_current = battery->get_current();
    state.battery_voltage = battery->get_voltage();

    return actual_motor_torque;
  }

public:
  FSFEUP03StandaloneModel(const InvictaSimParameters& p) {
    car_params = p.car_parameters;
    tire_model = tire_models_map.at(p.tire_model)(car_params);
    motor = motor_models_map.at(p.motor_model)(car_params);
    battery = battery_models_map.at(p.battery_model)(car_params);
    transmission = transmission_models_map.at(p.transmission_model)(car_params);
    inverter = inverter_models_map.at(p.inverter_model)(car_params);
    brake = brake_models_map.at(p.brake_model)(car_params);
    aero = aero_models_map.at(p.aero_model)(car_params);
    load_transfer = load_transfer_models_map.at(p.load_transfer_model)(car_params);
    steering = steering_models_map.at(p.steering_model)(car_params);
    steering_motor = steering_motor_models_map.at(p.steering_motor_model)(car_params);
  }

  void step(double dt, const common_lib::structures::Wheels& throttle, double steering_angle, SimState& state) override {
    double throttle_input = (throttle.rear_left + throttle.rear_right) / 2.0;

    common_lib::structures::Wheels brake_torques;
    double inverter_command = throttle_input;

    const double motor_torque = calculate_powertrain_torque(
        inverter->calculate_inverter_throttle(inverter_command, dt), dt, state
    );

    state.wheels_torque =
        transmission->calculate_wheel_torques(motor_torque, common_lib::structures::Wheels(state.wl_fl, state.wl_fr, state.wl_rl, state.wl_rr));

    const Eigen::Vector3d aero_forces =
        aero->aero_forces(Eigen::Vector3d(state.vx, state.vy, state.yaw_rate));
    state.aero_drag = aero_forces[0];
    state.aero_downforce = aero_forces[2];

    const double steering_rate =
        steering_motor->compute_steering_rate(state.steering_angle, steering_angle);
    state.steering_angle += steering_rate * dt;

    auto st_angles = steering->calculate_steering_angles(state.steering_angle);
    double actual_steering_fl = st_angles[0];
    double actual_steering_fr = st_angles[1];

    state.wheels_vertical_load = load_transfer->compute_loads(
        LoadTransferInput{state.ax, state.ay, aero_forces[2]}
    );

    TireInput tire_input;
    tire_input.dt = dt;
    tire_input.vx = state.vx;
    tire_input.vy = state.vy;
    tire_input.yaw_rate = state.yaw_rate;
    tire_input.last_slip_ratio = state.last_slip_ratio;
    tire_input.last_slip_angle = state.last_slip_angle;

    // FL
    tire_input.tire = FL;
    tire_input.steering_angle = actual_steering_fl;
    tire_input.wheel_angular_speed = state.wl_fl;
    tire_input.vertical_load = state.wheels_vertical_load.front_left;
    auto fl_forces = tire_model->calculate_tire_forces(tire_input);
    state.last_slip_ratio(FL) = tire_input.slip_ratio;
    state.last_slip_angle(FL) = tire_input.slip_angle;

    // FR
    tire_input.tire = FR;
    tire_input.steering_angle = actual_steering_fr;
    tire_input.wheel_angular_speed = state.wl_fr;
    tire_input.vertical_load = state.wheels_vertical_load.front_right;
    auto fr_forces = tire_model->calculate_tire_forces(tire_input);
    state.last_slip_ratio(FR) = tire_input.slip_ratio;
    state.last_slip_angle(FR) = tire_input.slip_angle;

    // RL
    tire_input.tire = RL;
    tire_input.steering_angle = 0.0;
    tire_input.wheel_angular_speed = state.wl_rl;
    tire_input.vertical_load = state.wheels_vertical_load.rear_left;
    auto rl_forces = tire_model->calculate_tire_forces(tire_input);
    state.last_slip_ratio(RL) = tire_input.slip_ratio;
    state.last_slip_angle(RL) = tire_input.slip_angle;

    // RR
    tire_input.tire = RR;
    tire_input.steering_angle = 0.0;
    tire_input.wheel_angular_speed = state.wl_rr;
    tire_input.vertical_load = state.wheels_vertical_load.rear_right;
    auto rr_forces = tire_model->calculate_tire_forces(tire_input);
    state.last_slip_ratio(RR) = tire_input.slip_ratio;
    state.last_slip_angle(RR) = tire_input.slip_angle;

    // Update wheel speeds
    double R = car_params->tire_parameters->effective_tire_r;
    double I = car_params->tire_parameters->wheel_inertia;
    const double brake_sign_fl = 2.0 / M_PI * std::atan(10.0 * state.wl_fl);
    const double brake_sign_fr = 2.0 / M_PI * std::atan(10.0 * state.wl_fr);
    const double brake_sign_rl = 2.0 / M_PI * std::atan(10.0 * state.wl_rl);
    const double brake_sign_rr = 2.0 / M_PI * std::atan(10.0 * state.wl_rr);

    state.wl_rl += ((state.wheels_torque.rear_left - (rl_forces[0] * R) - rl_forces[2] - brake_torques.rear_left * brake_sign_rl) / I) * dt;
    state.wl_rr += ((state.wheels_torque.rear_right - (rr_forces[0] * R) - rr_forces[2] - brake_torques.rear_right * brake_sign_rr) / I) * dt;
    state.wl_fl += ((-(fl_forces[0] * R) - (car_params->front_bearing_drag * state.wl_fl) - fl_forces[2] - brake_torques.front_left * brake_sign_fl) / I) * dt;
    state.wl_fr += ((-(fr_forces[0] * R) - (car_params->front_bearing_drag * state.wl_fr) - fr_forces[2] - brake_torques.front_right * brake_sign_fr) / I) * dt;

    double Fx_fl = fl_forces[0] * cos(actual_steering_fl) - fl_forces[1] * sin(actual_steering_fl);
    double Fy_fl = fl_forces[0] * sin(actual_steering_fl) + fl_forces[1] * cos(actual_steering_fl);
    double Fx_fr = fr_forces[0] * cos(actual_steering_fr) - fr_forces[1] * sin(actual_steering_fr);
    double Fy_fr = fr_forces[0] * sin(actual_steering_fr) + fr_forces[1] * cos(actual_steering_fr);

    double total_fx = Fx_fl + Fx_fr + rl_forces[0] + rr_forces[0] + aero_forces[0];
    double total_fy = Fy_fl + Fy_fr + rl_forces[1] + rr_forces[1] + aero_forces[1];

    state.ax = total_fx / car_params->total_mass + state.vy * state.yaw_rate;
    state.ay = total_fy / car_params->total_mass - state.vx * state.yaw_rate;

    state.vx += state.ax * dt;
    state.vy += state.ay * dt;

    double speed = std::sqrt(state.vx * state.vx + state.vy * state.vy);
    if (speed < 0.05 && std::abs(throttle_input) < 0.01) {
      state.vx = 0.0;
      state.vy = 0.0;
      state.ax = 0.0;
      state.ay = 0.0;
      state.yaw_rate = 0.0;
      state.wl_fl = 0.0;
      state.wl_fr = 0.0;
      state.wl_rl = 0.0;
      state.wl_rr = 0.0;
      state.last_slip_ratio.setZero();
      state.last_slip_angle.setZero();
    }

    double lr = car_params->cg_2_rear_axis;
    double lf = car_params->wheelbase - lr;
    double half_width = car_params->track_width / 2.0;

    double moment_fy = (Fy_fl + Fy_fr) * lf - (rl_forces[1] + rr_forces[1]) * lr;
    double moment_fx = (Fx_fr - Fx_fl) * half_width + (rr_forces[0] - rl_forces[0]) * half_width;
    double total_mz = fl_forces[3] + fr_forces[3] + rl_forces[3] + rr_forces[3];

    double total_torque = moment_fy + moment_fx + total_mz;

    double yaw_a = total_torque / car_params->Izz;
    state.yaw_rate += yaw_a * dt;
    state.yaw += state.yaw_rate * dt;

    if (state.yaw > M_PI) state.yaw -= 2.0 * M_PI;
    if (state.yaw < -M_PI) state.yaw += 2.0 * M_PI;

    double cos_yaw = cos(state.yaw);
    double sin_yaw = sin(state.yaw);
    state.x += (state.vx * cos_yaw - state.vy * sin_yaw) * dt;
    state.y += (state.vx * sin_yaw + state.vy * cos_yaw) * dt;
  }
};

std::unique_ptr<StandaloneVehicleModel> create_vehicle_model(const InvictaSimParameters& p) {
  if (p.vehicle_model == "fsfeup02") {
    return std::make_unique<FSFEUP02StandaloneModel>(p);
  } else if (p.vehicle_model == "fsfeup03") {
    return std::make_unique<FSFEUP03StandaloneModel>(p);
  } else {
    std::cerr << "Warning: Unknown vehicle model '" << p.vehicle_model << "', defaulting to fsfeup02." << std::endl;
    return std::make_unique<FSFEUP02StandaloneModel>(p);
  }
}

std::shared_ptr<common_lib::car_parameters::CarParameters> deep_copy_car_parameters(
    std::shared_ptr<common_lib::car_parameters::CarParameters> src) {
  if (!src) return nullptr;
  auto dst = std::make_shared<common_lib::car_parameters::CarParameters>(*src);
  if (src->aero_parameters) dst->aero_parameters = std::make_shared<common_lib::car_parameters::AeroParameters>(*src->aero_parameters);
  if (src->battery_parameters) dst->battery_parameters = std::make_shared<common_lib::car_parameters::BatteryParameters>(*src->battery_parameters);
  if (src->brake_parameters) dst->brake_parameters = std::make_shared<common_lib::car_parameters::BrakeParameters>(*src->brake_parameters);
  if (src->inverter_parameters) dst->inverter_parameters = std::make_shared<common_lib::car_parameters::InverterParameters>(*src->inverter_parameters);
  if (src->load_transfer_parameters) dst->load_transfer_parameters = std::make_shared<common_lib::car_parameters::LoadTransferParameters>(*src->load_transfer_parameters);
  if (src->motor_parameters) dst->motor_parameters = std::make_shared<common_lib::car_parameters::MotorParameters>(*src->motor_parameters);
  if (src->steering_motor_parameters) dst->steering_motor_parameters = std::make_shared<common_lib::car_parameters::SteeringMotorParameters>(*src->steering_motor_parameters);
  if (src->steering_parameters) dst->steering_parameters = std::make_shared<common_lib::car_parameters::SteeringParameters>(*src->steering_parameters);
  if (src->tire_parameters) dst->tire_parameters = std::make_shared<common_lib::car_parameters::TireParameters>(*src->tire_parameters);
  if (src->transmission_parameters) dst->transmission_parameters = std::make_shared<common_lib::car_parameters::TransmissionParameters>(*src->transmission_parameters);
  return dst;
}

double evaluate_candidate(
    const std::vector<std::vector<CsvRow>>& all_csvs_rows,
    const std::vector<ParameterSpec>& param_specs,
    const std::vector<double>& candidate_values,
    const YAML::Node& tuning_config,
    const InvictaSimParameters& base_params
) {
  InvictaSimParameters sim_params = base_params;
  sim_params.car_parameters = deep_copy_car_parameters(base_params.car_parameters);

  for (size_t i = 0; i < param_specs.size(); ++i) {
    apply_parameter_override(sim_params.car_parameters, param_specs[i].name, candidate_values[i]);
  }

  YAML::Node csvs = tuning_config["tuning"]["csvs"];
  if (!csvs && tuning_config["tuning"]["bags"]) {
    csvs = tuning_config["tuning"]["bags"];
  }
  YAML::Node default_score_config = tuning_config["tuning"]["score"];

  double weighted_score_sum = 0.0;
  double total_weight = 0.0;

  for (size_t b = 0; b < csvs.size(); ++b) {
    YAML::Node csv_node = csvs[b];
    double weight = csv_node["weight"] ? csv_node["weight"].as<double>() : 1.0;
    double start_offset = csv_node["start_offset_s"] ? csv_node["start_offset_s"].as<double>() : 0.0;
    double stop_duration = csv_node["stop_after_s"] ? csv_node["stop_after_s"].as<double>() : -1.0;

    YAML::Node score_config = default_score_config;
    if (csv_node["mission"]) {
      std::string mission_name = csv_node["mission"].as<std::string>();
      if (tuning_config["tuning"]["missions"] && tuning_config["tuning"]["missions"][mission_name]) {
        score_config = tuning_config["tuning"]["missions"][mission_name];
      } else {
        std::cerr << "Warning: Mission '" << mission_name << "' not found in tuning:missions. Falling back to default." << std::endl;
      }
    } else if (csv_node["score"]) {
      score_config = csv_node["score"];
    }

    const auto& rows = all_csvs_rows[b];
    if (rows.empty()) {
      weighted_score_sum += weight * 1e9;
      total_weight += weight;
      continue;
    }

    size_t start_idx = 0;
    for (size_t i = 0; i < rows.size(); ++i) {
      if (rows[i].timestamp_s >= start_offset) {
        start_idx = i;
        break;
      }
    }

    double start_time = rows[start_idx].timestamp_s;

    // Instantiate polymorphic vehicle model based on config selection
    std::unique_ptr<StandaloneVehicleModel> model = create_vehicle_model(sim_params);

    SimState state;
    CsvRow first_sample = rows[start_idx];
    state.x = first_sample.real_x;
    state.y = first_sample.real_y;
    state.yaw = first_sample.real_yaw;
    state.vx = first_sample.real_vx;
    state.vy = first_sample.real_vy;
    state.yaw_rate = first_sample.real_yaw_rate;
    state.wl_fl = first_sample.real_fl_rpm * 2.0 * M_PI / 60.0;
    state.wl_fr = first_sample.real_fr_rpm * 2.0 * M_PI / 60.0;
    state.wl_rl = first_sample.real_rl_rpm * 2.0 * M_PI / 60.0;
    state.wl_rr = first_sample.real_rr_rpm * 2.0 * M_PI / 60.0;
    state.motor_omega = first_sample.real_motor_rpm * 2.0 * M_PI / 60.0;

    RunningRmse position_rmse;
    RunningRmse heading_rmse;
    RunningRmse velocity_rmse;
    RunningRmse velocity_x_rmse;
    RunningRmse velocity_x_slow_rmse;
    RunningRmse velocity_y_rmse;
    RunningRmse yaw_rate_rmse;
    RunningRmse yaw_rate_under_rmse;
    RunningRmse front_wheel_rpm_rmse;
    RunningRmse motor_rpm_rmse;

    PoseSample real_origin = {first_sample.real_x, first_sample.real_y, first_sample.real_yaw};
    PoseSample sim_origin = {state.x, state.y, state.yaw};

    double last_time = first_sample.timestamp_s;

    for (size_t i = start_idx + 1; i < rows.size(); ++i) {
      const CsvRow& row = rows[i];
      if (stop_duration > 0.0 && (row.timestamp_s - start_time) > stop_duration) {
        break;
      }
      double dt = row.timestamp_s - last_time;
      if (dt <= 0.0) continue;
      last_time = row.timestamp_s;

      const CsvRow& prev_row = rows[i - 1];
      common_lib::structures::Wheels throttle(prev_row.throttle_fl, prev_row.throttle_fr, prev_row.throttle_rl, prev_row.throttle_rr);

      model->step(dt, throttle, prev_row.steering, state);

      double sim_fl_rpm = state.wl_fl * 60.0 / (2.0 * M_PI);
      double sim_fr_rpm = state.wl_fr * 60.0 / (2.0 * M_PI);
      double sim_front_rpm = 0.5 * (sim_fl_rpm + sim_fr_rpm);
      double sim_motor_rpm = state.motor_omega * 60.0 / (2.0 * M_PI);

      double real_front_rpm = 0.5 * (row.real_fl_rpm + row.real_fr_rpm);

      PoseSample real_raw = {row.real_x, row.real_y, row.real_yaw};
      PoseSample real_pose = transform_pose_to_map(real_raw, real_origin, sim_origin);

      double pos_err = std::hypot(state.x - real_pose.x, state.y - real_pose.y);
      double head_err = normalize_angle(state.yaw - real_pose.yaw);
      double vel_x_err = state.vx - row.real_vx;
      double vel_y_err = state.vy - row.real_vy;
      double vel_err = std::hypot(vel_x_err, vel_y_err);
      double yaw_err = state.yaw_rate - row.real_yaw_rate;
      double front_rpm_err = sim_front_rpm - real_front_rpm;
      double motor_rpm_err = sim_motor_rpm - row.real_motor_rpm;

      position_rmse.update(pos_err);
      heading_rmse.update(head_err);
      velocity_rmse.update(vel_err);
      velocity_x_rmse.update(vel_x_err);
      velocity_x_slow_rmse.update(std::min(0.0, vel_x_err));
      velocity_y_rmse.update(vel_y_err);
      yaw_rate_rmse.update(yaw_err);
      yaw_rate_under_rmse.update(std::min(0.0, yaw_err));
      front_wheel_rpm_rmse.update(front_rpm_err);
      motor_rpm_rmse.update(motor_rpm_err);
    }

    int samples = position_rmse.count;
    double dataset_score = 0.0;
    YAML::Node weights = score_config["weights"];
    if (weights) {
      if (weights["position_rmse"]) dataset_score += weights["position_rmse"].as<double>() * position_rmse.get();
      if (weights["heading_rmse"]) dataset_score += weights["heading_rmse"].as<double>() * heading_rmse.get();
      if (weights["velocity_rmse"]) dataset_score += weights["velocity_rmse"].as<double>() * velocity_rmse.get();
      if (weights["velocity_x_rmse"]) dataset_score += weights["velocity_x_rmse"].as<double>() * velocity_x_rmse.get();
      if (weights["velocity_x_slow_rmse"]) dataset_score += weights["velocity_x_slow_rmse"].as<double>() * velocity_x_slow_rmse.get();
      if (weights["velocity_y_rmse"]) dataset_score += weights["velocity_y_rmse"].as<double>() * velocity_y_rmse.get();
      if (weights["yaw_rate_rmse"]) dataset_score += weights["yaw_rate_rmse"].as<double>() * yaw_rate_rmse.get();
      if (weights["yaw_rate_under_rmse"]) dataset_score += weights["yaw_rate_under_rmse"].as<double>() * yaw_rate_under_rmse.get();
      if (weights["front_wheel_rpm_rmse"]) dataset_score += weights["front_wheel_rpm_rmse"].as<double>() * front_wheel_rpm_rmse.get();
      if (weights["motor_rpm_rmse"]) dataset_score += weights["motor_rpm_rmse"].as<double>() * motor_rpm_rmse.get();
    }

    int min_samples = score_config["min_samples"] ? score_config["min_samples"].as<int>() : 0;
    if (min_samples > 0 && samples < min_samples) {
      double shortfall = static_cast<double>(min_samples - samples) / static_cast<double>(min_samples);
      double penalty = score_config["short_sample_penalty"] ? score_config["short_sample_penalty"].as<double>() : 0.0;
      dataset_score += penalty * shortfall;
    }

    weighted_score_sum += weight * dataset_score;
    total_weight += weight;
  }

  return weighted_score_sum / std::max(total_weight, 1e-9);
}

std::string get_full_csv_path(const std::string& data_dir, const std::string& rel_path) {
  if (std::filesystem::path(rel_path).is_absolute() || data_dir.empty()) {
    return rel_path;
  }
  return (std::filesystem::path(data_dir) / rel_path).string();
}

Individual run_genetic_algorithm(
    const std::vector<std::vector<CsvRow>>& all_csvs_rows,
    const std::vector<ParameterSpec>& param_specs,
    const YAML::Node& tuning_config,
    const InvictaSimParameters& base_params
) {
  YAML::Node opt = tuning_config["tuning"]["optimizer"];
  int generations = opt["generations"] ? opt["generations"].as<int>() : 15;
  int population_size = opt["population_size"] ? opt["population_size"].as<int>() : 24;
  int elite_count = opt["elite_count"] ? opt["elite_count"].as<int>() : 4;
  int immigrant_count = opt["immigrant_count"] ? opt["immigrant_count"].as<int>() : 3;
  double mutation_rate = opt["mutation_rate"] ? opt["mutation_rate"].as<double>() : 0.35;
  double mutation_scale = opt["mutation_scale"] ? opt["mutation_scale"].as<double>() : 0.16;
  double blend_alpha = opt["blend_alpha"] ? opt["blend_alpha"].as<double>() : 0.25;
  int tournament_size = opt["tournament_size"] ? opt["tournament_size"].as<int>() : 3;
  int seed = opt["seed"] ? opt["seed"].as<int>() : 42;

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist_01(0.0, 1.0);

  // Initialize population
  std::vector<double> baseline_vals(param_specs.size());
  for (size_t p = 0; p < param_specs.size(); ++p) {
    baseline_vals[p] = get_baseline_value(base_params.car_parameters, param_specs[p].name);
  }

  std::vector<Individual> population(population_size);
  population[0].values = baseline_vals;

  for (int i = 1; i < population_size; ++i) {
    population[i].values.resize(param_specs.size());
    for (size_t p = 0; p < param_specs.size(); ++p) {
      std::uniform_real_distribution<double> dist(param_specs[p].min_val, param_specs[p].max_val);
      population[i].values[p] = dist(rng);
    }
  }

  Individual global_best;
  global_best.score = 1e9;
  global_best.values = population[0].values;

  for (int gen = 0; gen < generations; ++gen) {
    std::cout << "--- Generation " << gen + 1 << "/" << generations << " ---" << std::endl;

    // Evaluate population in parallel using std::async
    std::vector<std::future<double>> futures(population_size);
    for (int i = 0; i < population_size; ++i) {
      futures[i] = std::async(std::launch::async, [&, i]() {
        return evaluate_candidate(all_csvs_rows, param_specs, population[i].values, tuning_config, base_params);
      });
    }

    for (int i = 0; i < population_size; ++i) {
      population[i].score = futures[i].get();
      if (population[i].score < global_best.score) {
        global_best = population[i];
        std::cout << "  [New Global Best] Score: " << global_best.score << " Parameters:";
        for (size_t p = 0; p < param_specs.size(); ++p) {
          std::cout << " " << param_specs[p].name << "=" << global_best.values[p];
        }
        std::cout << std::endl;
      }
    }

    // Sort by score
    std::sort(population.begin(), population.end(), [](const Individual& a, const Individual& b) {
      return a.score < b.score;
    });

    std::cout << "  Best score in generation: " << population[0].score << std::endl;

    std::vector<Individual> next_pop;
    next_pop.reserve(population_size);

    // Keep elites
    for (int i = 0; i < std::min(elite_count, population_size); ++i) {
      next_pop.push_back(population[i]);
    }

    // Add immigrants
    for (int i = 0; i < immigrant_count; ++i) {
      Individual immigrant;
      immigrant.values.resize(param_specs.size());
      for (size_t p = 0; p < param_specs.size(); ++p) {
        std::uniform_real_distribution<double> dist(param_specs[p].min_val, param_specs[p].max_val);
        immigrant.values[p] = dist(rng);
      }
      next_pop.push_back(immigrant);
    }

    auto tournament_select = [&](int size) -> const Individual& {
      int best_idx = rng() % population_size;
      for (int t = 1; t < size; ++t) {
        int idx = rng() % population_size;
        if (population[idx].score < population[best_idx].score) {
          best_idx = idx;
        }
      }
      return population[best_idx];
    };

    // Fill remaining
    while (next_pop.size() < static_cast<size_t>(population_size)) {
      const Individual& parent_a = tournament_select(tournament_size);
      const Individual& parent_b = tournament_select(tournament_size);

      Individual child;
      child.values.resize(param_specs.size());
      for (size_t p = 0; p < param_specs.size(); ++p) {
        std::uniform_real_distribution<double> dist_blend(-blend_alpha, 1.0 + blend_alpha);
        double alpha = dist_blend(rng);
        double val = alpha * parent_a.values[p] + (1.0 - alpha) * parent_b.values[p];
        child.values[p] = std::clamp(val, param_specs[p].min_val, param_specs[p].max_val);
      }

      // Mutate
      for (size_t p = 0; p < param_specs.size(); ++p) {
        if (dist_01(rng) < mutation_rate) {
          double span = param_specs[p].max_val - param_specs[p].min_val;
          std::normal_distribution<double> dist_normal(0.0, mutation_scale * span);
          child.values[p] = std::clamp(child.values[p] + dist_normal(rng), param_specs[p].min_val, param_specs[p].max_val);
        }
      }

      next_pop.push_back(child);
    }

    population = next_pop;
  }

  return global_best;
}



int main(int argc, char** argv) {
  std::string config_path;
  std::string data_dir;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--data-dir" && i + 1 < argc) {
      data_dir = argv[++i];
    }
  }

  if (config_path.empty() || data_dir.empty()) {
    std::cerr << "Usage: " << argv[0] << " --config <config_yaml> --data-dir <data_csv_directory>" << std::endl;
    return 1;
  }

  std::cout << "Loading tuning configuration from: " << config_path << std::endl;
  YAML::Node tuning_config = YAML::LoadFile(config_path);

  rclcpp::init(0, nullptr);
  InvictaSimParameters base_params;

  YAML::Node csvs = tuning_config["tuning"]["csvs"];
  if (!csvs && tuning_config["tuning"]["bags"]) {
    csvs = tuning_config["tuning"]["bags"];
  }
  std::vector<std::vector<CsvRow>> all_csvs_rows;
  all_csvs_rows.reserve(csvs.size());

  std::cout << "Loading CSV telemetry datasets..." << std::endl;
  for (size_t b = 0; b < csvs.size(); ++b) {
    std::string full_path;
    if (csvs[b]["csv_path"]) {
      full_path = get_full_csv_path(data_dir, csvs[b]["csv_path"].as<std::string>());
    } else if (csvs[b]["name"]) {
      full_path = (std::filesystem::path(data_dir) / (csvs[b]["name"].as<std::string>() + ".csv")).string();
    } else if (csvs[b]["path"]) {
      std::string rel_path = csvs[b]["path"].as<std::string>();
      full_path = get_full_csv_path(data_dir, rel_path);
    } else {
      std::cerr << "Warning: Bag entry is missing path or name." << std::endl;
      continue;
    }
    std::cout << "  Reading: " << full_path << std::endl;
    auto rows = read_csv(full_path);
    std::cout << "    Loaded " << rows.size() << " samples." << std::endl;
    all_csvs_rows.push_back(std::move(rows));
  }

  std::vector<ParameterSpec> param_specs;
  YAML::Node params_node = tuning_config["tuning"]["parameters"];
  if (params_node) {
    for (size_t i = 0; i < params_node.size(); ++i) {
      ParameterSpec spec;
      spec.name = params_node[i]["name"].as<std::string>();
      spec.min_val = params_node[i]["min"].as<double>();
      spec.max_val = params_node[i]["max"].as<double>();
      param_specs.push_back(spec);
    }
  }

  std::cout << "Loaded " << param_specs.size() << " optimization parameters." << std::endl;
  std::cout << "Selected vehicle model from configuration: " << base_params.vehicle_model << std::endl;

  std::cout << "Starting Genetic Algorithm search..." << std::endl;
  auto start_time = std::chrono::steady_clock::now();
  Individual optimal = run_genetic_algorithm(all_csvs_rows, param_specs, tuning_config, base_params);
  auto end_time = std::chrono::steady_clock::now();

  double elapsed_s = std::chrono::duration<double>(end_time - start_time).count();
  std::cout << "\n========================================" << std::endl;
  std::cout << "Optimization Complete in " << elapsed_s << " seconds." << std::endl;
  std::cout << "Optimal Cost Score: " << optimal.score << std::endl;
  std::cout << "Optimal Parameters:" << std::endl;
  for (size_t p = 0; p < param_specs.size(); ++p) {
    std::cout << "  " << param_specs[p].name << ": " << optimal.values[p] << std::endl;
  }
  std::cout << "========================================" << std::endl;

  rclcpp::shutdown();
  return 0;
}
