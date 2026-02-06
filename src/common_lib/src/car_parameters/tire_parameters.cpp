#include "common_lib/car_parameters/tire_parameters.hpp"

namespace common_lib::car_parameters {

TireParameters::TireParameters(const std::string& config_name) {
  std::string config_path = common_lib::config_load::get_config_yaml_path(
      "common_lib", "motion_lib/tire_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  tire_B_lateral = config["tire_B_lateral"].as<double>();
  tire_C_lateral = config["tire_C_lateral"].as<double>();
  tire_D_lateral = config["tire_D_lateral"].as<double>();
  tire_E_lateral = config["tire_E_lateral"].as<double>();
  tire_B_longitudinal = config["tire_B_longitudinal"].as<double>();
  tire_C_longitudinal = config["tire_C_longitudinal"].as<double>();
  tire_D_longitudinal = config["tire_D_longitudinal"].as<double>();
  tire_E_longitudinal = config["tire_E_longitudinal"].as<double>();

  camber_scaling_factor = config["camber_scaling_factor"].as<double>();
  effective_tire_r = config["effective_tire_r"].as<double>();
  distance_to_CG = config["distance_to_CG"].as<double>();

  d_bright = config["d_bright"].as<double>();
  d_bleft = config["d_bleft"].as<double>();
  d_fright = config["d_fright"].as<double>();
  d_fleft = config["d_fleft"].as<double>();

  camber = config["camber"].as<double>();

  UNLOADED_RADIUS = config["UNLOADED_RADIUS"].as<double>();
  WIDTH = config["WIDTH"].as<double>();
  ASPECT_RATIO = config["ASPECT_RATIO"].as<double>();
  RIM_RADIUS = config["RIM_RADIUS"].as<double>();
  RIM_WIDTH = config["RIM_WIDTH"].as<double>();

  INFLPRES = config["INFLPRES"].as<double>();
  NOMPRES = config["NOMPRES"].as<double>();

  MASS = config["MASS"].as<double>();
  IXX = config["IXX"].as<double>();
  IYY = config["IYY"].as<double>();
  BELT_MASS = config["BELT_MASS"].as<double>();
  BELT_IXX = config["BELT_IXX"].as<double>();
  BELT_IYY = config["BELT_IYY"].as<double>();
  GRAVITY = config["GRAVITY"].as<double>();

  FNOMIN = config["FNOMIN"].as<double>();
  VERTICAL_STIFFNESS = config["VERTICAL_STIFFNESS"].as<double>();
  VERTICAL_DAMPING = config["VERTICAL_DAMPING"].as<double>();
  MC_CONTOUR_A = config["MC_CONTOUR_A"].as<double>();
  MC_CONTOUR_B = config["MC_CONTOUR_B"].as<double>();
  BREFF = config["BREFF"].as<double>();
  DREFF = config["DREFF"].as<double>();
  FREFF = config["FREFF"].as<double>();
  Q_RE0 = config["Q_RE0"].as<double>();
  Q_V1 = config["Q_V1"].as<double>();
  Q_V2 = config["Q_V2"].as<double>();
  Q_FZ2 = config["Q_FZ2"].as<double>();
  Q_FCX = config["Q_FCX"].as<double>();
  Q_FCY = config["Q_FCY"].as<double>();
  Q_CAM = config["Q_CAM"].as<double>();
  PFZ1 = config["PFZ1"].as<double>();
  Q_FCY2 = config["Q_FCY2"].as<double>();
  Q_CAM1 = config["Q_CAM1"].as<double>();
  Q_CAM2 = config["Q_CAM2"].as<double>();
  Q_CAM3 = config["Q_CAM3"].as<double>();
  Q_FYS1 = config["Q_FYS1"].as<double>();
  Q_FYS2 = config["Q_FYS2"].as<double>();
  Q_FYS3 = config["Q_FYS3"].as<double>();
  BOTTOM_OFFST = config["BOTTOM_OFFST"].as<double>();
  BOTTOM_STIFF = config["BOTTOM_STIFF"].as<double>();

  // Structural
  LONGITUDINAL_STIFFNESS = config["LONGITUDINAL_STIFFNESS"].as<double>();
  LATERAL_STIFFNESS = config["LATERAL_STIFFNESS"].as<double>();
  YAW_STIFFNESS = config["YAW_STIFFNESS"].as<double>();
  FREQ_LONG = config["FREQ_LONG"].as<double>();
  FREQ_LAT = config["FREQ_LAT"].as<double>();
  FREQ_YAW = config["FREQ_YAW"].as<double>();
  FREQ_WINDUP = config["FREQ_WINDUP"].as<double>();
  DAMP_LONG = config["DAMP_LONG"].as<double>();
  DAMP_LAT = config["DAMP_LAT"].as<double>();
  DAMP_YAW = config["DAMP_YAW"].as<double>();
  DAMP_WINDUP = config["DAMP_WINDUP"].as<double>();
  DAMP_RESIDUAL = config["DAMP_RESIDUAL"].as<double>();
  DAMP_VLOW = config["DAMP_VLOW"].as<double>();
  Q_BVX = config["Q_BVX"].as<double>();
  Q_BVT = config["Q_BVT"].as<double>();
  PCFX1 = config["PCFX1"].as<double>();
  PCFX2 = config["PCFX2"].as<double>();
  PCFX3 = config["PCFX3"].as<double>();
  PCFY1 = config["PCFY1"].as<double>();
  PCFY2 = config["PCFY2"].as<double>();
  PCFY3 = config["PCFY3"].as<double>();
  PCMZ1 = config["PCMZ1"].as<double>();

  // Contact patch
  Q_RA1 = config["Q_RA1"].as<double>();
  Q_RA2 = config["Q_RA2"].as<double>();
  Q_RB1 = config["Q_RB1"].as<double>();
  Q_RB2 = config["Q_RB2"].as<double>();
  ELLIPS_SHIFT = config["ELLIPS_SHIFT"].as<double>();
  ELLIPS_LENGTH = config["ELLIPS_LENGTH"].as<double>();
  ELLIPS_HEIGHT = config["ELLIPS_HEIGHT"].as<double>();
  ELLIPS_ORDER = config["ELLIPS_ORDER"].as<double>();
  ELLIPS_MAX_STEP = config["ELLIPS_MAX_STEP"].as<double>();
  ELLIPS_NWIDTH = config["ELLIPS_NWIDTH"].as<double>();
  ELLIPS_NLENGTH = config["ELLIPS_NLENGTH"].as<double>();

  // Inflation pressure range
  PRESMIN = config["PRESMIN"].as<double>();
  PRESMAX = config["PRESMAX"].as<double>();

  // Vertical force range
  FZMIN = config["FZMIN"].as<double>();
  FZMAX = config["FZMAX"].as<double>();

  // Long slip range
  KPUMIN = config["KPUMIN"].as<double>();
  KPUMAX = config["KPUMAX"].as<double>();

  // Slip angle range
  ALPMIN = config["ALPMIN"].as<double>();
  ALPMAX = config["ALPMAX"].as<double>();

  // Inclination angle range
  CAMMIN = config["CAMMIN"].as<double>();
  CAMMAX = config["CAMMAX"].as<double>();

  // Scaling coefficients
  LFZO = config["LFZO"].as<double>();
  LCX = config["LCX"].as<double>();
  LMUX = config["LMUX"].as<double>();
  LEX = config["LEX"].as<double>();
  LKX = config["LKX"].as<double>();
  LHX = config["LHX"].as<double>();
  LVX = config["LVX"].as<double>();
  LCY = config["LCY"].as<double>();
  LMUY = config["LMUY"].as<double>();
  LEY = config["LEY"].as<double>();
  LKY = config["LKY"].as<double>();
  LHY = config["LHY"].as<double>();
  LVY = config["LVY"].as<double>();
  LTR = config["LTR"].as<double>();
  LRES = config["LRES"].as<double>();
  LXAL = config["LXAL"].as<double>();
  LYKA = config["LYKA"].as<double>();
  LVYKA = config["LVYKA"].as<double>();
  LS = config["LS"].as<double>();
  LKYC = config["LKYC"].as<double>();
  LKZC = config["LKZC"].as<double>();
  LVMX = config["LVMX"].as<double>();
  LMX = config["LMX"].as<double>();
  LMY = config["LMY"].as<double>();
  LMP = config["LMP"].as<double>();

  // Longitudinal coefficients
  PCX1 = config["PCX1"].as<double>();
  PDX1 = config["PDX1"].as<double>();
  PDX2 = config["PDX2"].as<double>();
  PDX3 = config["PDX3"].as<double>();
  PEX1 = config["PEX1"].as<double>();
  PEX2 = config["PEX2"].as<double>();
  PEX3 = config["PEX3"].as<double>();
  PEX4 = config["PEX4"].as<double>();
  PKX1 = config["PKX1"].as<double>();
  PKX2 = config["PKX2"].as<double>();
  PKX3 = config["PKX3"].as<double>();
  PHX1 = config["PHX1"].as<double>();
  PHX2 = config["PHX2"].as<double>();
  PVX1 = config["PVX1"].as<double>();
  PVX2 = config["PVX2"].as<double>();
  PPX1 = config["PPX1"].as<double>();
  PPX2 = config["PPX2"].as<double>();
  PPX3 = config["PPX3"].as<double>();
  PPX4 = config["PPX4"].as<double>();
  RBX1 = config["RBX1"].as<double>();
  RBX2 = config["RBX2"].as<double>();
  RBX3 = config["RBX3"].as<double>();
  RCX1 = config["RCX1"].as<double>();
  REX1 = config["REX1"].as<double>();
  REX2 = config["REX2"].as<double>();
  RHX1 = config["RHX1"].as<double>();

  // Overturning coefficients
  QSX1 = config["QSX1"].as<double>();
  QSX2 = config["QSX2"].as<double>();
  QSX3 = config["QSX3"].as<double>();
  QSX4 = config["QSX4"].as<double>();
  QSX5 = config["QSX5"].as<double>();
  QSX6 = config["QSX6"].as<double>();
  QSX7 = config["QSX7"].as<double>();
  QSX8 = config["QSX8"].as<double>();
  QSX9 = config["QSX9"].as<double>();
  QSX10 = config["QSX10"].as<double>();
  QSX11 = config["QSX11"].as<double>();
  QSX12 = config["QSX12"].as<double>();
  QSX13 = config["QSX13"].as<double>();
  QSX14 = config["QSX14"].as<double>();
  PPMX1 = config["PPMX1"].as<double>();

  // Lateral coefficients
  PCY1 = config["PCY1"].as<double>();
  PDY1 = config["PDY1"].as<double>();
  PDY2 = config["PDY2"].as<double>();
  PDY3 = config["PDY3"].as<double>();
  PEY1 = config["PEY1"].as<double>();
  PEY2 = config["PEY2"].as<double>();
  PEY3 = config["PEY3"].as<double>();
  PEY4 = config["PEY4"].as<double>();
  PEY5 = config["PEY5"].as<double>();
  PKY1 = config["PKY1"].as<double>();
  PKY2 = config["PKY2"].as<double>();
  PKY3 = config["PKY3"].as<double>();
  PKY4 = config["PKY4"].as<double>();
  PKY5 = config["PKY5"].as<double>();
  PKY6 = config["PKY6"].as<double>();
  PKY7 = config["PKY7"].as<double>();
  PHY1 = config["PHY1"].as<double>();
  PHY2 = config["PHY2"].as<double>();
  PVY1 = config["PVY1"].as<double>();
  PVY2 = config["PVY2"].as<double>();
  PVY3 = config["PVY3"].as<double>();
  PVY4 = config["PVY4"].as<double>();
  PPY1 = config["PPY1"].as<double>();
  PPY2 = config["PPY2"].as<double>();
  PPY3 = config["PPY3"].as<double>();
  PPY4 = config["PPY4"].as<double>();
  PPY5 = config["PPY5"].as<double>();
  RBY1 = config["RBY1"].as<double>();
  RBY2 = config["RBY2"].as<double>();
  RBY3 = config["RBY3"].as<double>();
  RBY4 = config["RBY4"].as<double>();
  RCY1 = config["RCY1"].as<double>();
  REY1 = config["REY1"].as<double>();
  REY2 = config["REY2"].as<double>();
  RHY1 = config["RHY1"].as<double>();
  RHY2 = config["RHY2"].as<double>();
  RVY1 = config["RVY1"].as<double>();
  RVY2 = config["RVY2"].as<double>();
  RVY3 = config["RVY3"].as<double>();
  RVY4 = config["RVY4"].as<double>();
  RVY5 = config["RVY5"].as<double>();
  RVY6 = config["RVY6"].as<double>();

  // Rolling coefficients
  QSY1 = config["QSY1"].as<double>();
  QSY2 = config["QSY2"].as<double>();
  QSY3 = config["QSY3"].as<double>();
  QSY4 = config["QSY4"].as<double>();
  QSY5 = config["QSY5"].as<double>();
  QSY6 = config["QSY6"].as<double>();
  QSY7 = config["QSY7"].as<double>();
  QSY8 = config["QSY8"].as<double>();

  // Aligning coefficients
  QBZ1 = config["QBZ1"].as<double>();
  QBZ2 = config["QBZ2"].as<double>();
  QBZ3 = config["QBZ3"].as<double>();
  QBZ4 = config["QBZ4"].as<double>();
  QBZ5 = config["QBZ5"].as<double>();
  QBZ9 = config["QBZ9"].as<double>();
  QBZ10 = config["QBZ10"].as<double>();
  QCZ1 = config["QCZ1"].as<double>();
  QDZ1 = config["QDZ1"].as<double>();
  QDZ2 = config["QDZ2"].as<double>();
  QDZ3 = config["QDZ3"].as<double>();
  QDZ4 = config["QDZ4"].as<double>();
  QDZ6 = config["QDZ6"].as<double>();
  QDZ7 = config["QDZ7"].as<double>();
  QDZ8 = config["QDZ8"].as<double>();
  QDZ9 = config["QDZ9"].as<double>();
  QDZ10 = config["QDZ10"].as<double>();
  QDZ11 = config["QDZ11"].as<double>();
  QEZ1 = config["QEZ1"].as<double>();
  QEZ2 = config["QEZ2"].as<double>();
  QEZ3 = config["QEZ3"].as<double>();
  QEZ4 = config["QEZ4"].as<double>();
  QEZ5 = config["QEZ5"].as<double>();
  QHZ1 = config["QHZ1"].as<double>();
  QHZ2 = config["QHZ2"].as<double>();
  QHZ3 = config["QHZ3"].as<double>();
  QHZ4 = config["QHZ4"].as<double>();
  PPZ1 = config["PPZ1"].as<double>();
  PPZ2 = config["PPZ2"].as<double>();
  SSZ1 = config["SSZ1"].as<double>();
  SSZ2 = config["SSZ2"].as<double>();
  SSZ3 = config["SSZ3"].as<double>();
  SSZ4 = config["SSZ4"].as<double>();

  // Turnslip coefficients
  PECP1 = config["PECP1"].as<double>();
  PECP2 = config["PECP2"].as<double>();
  PDXP1 = config["PDXP1"].as<double>();
  PDXP2 = config["PDXP2"].as<double>();
  PDXP3 = config["PDXP3"].as<double>();
  PDXP4 = config["PDXP4"].as<double>();
  PDYP1 = config["PDYP1"].as<double>();
  PDYP2 = config["PDYP2"].as<double>();
  PDYP3 = config["PDYP3"].as<double>();
  PDYP4 = config["PDYP4"].as<double>();
  PKYP1 = config["PKYP1"].as<double>();
  PHYP1 = config["PHYP1"].as<double>();
  PHYP2 = config["PHYP2"].as<double>();
  PHYP3 = config["PHYP3"].as<double>();
  PHYP4 = config["PHYP4"].as<double>();

  // Configurations
  Amu = config["Amu"].as<double>();
}
}  // namespace common_lib::car_parameters
