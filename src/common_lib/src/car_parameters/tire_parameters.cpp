#include "common_lib/car_parameters/tire_parameters.hpp"

namespace common_lib::car_parameters {

TireParameters::TireParameters(const std::string& config_name) {
  std::string config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "car/tire_model", config_name);
  YAML::Node root = YAML::LoadFile(config_path);
  YAML::Node config = root["tire"];

  // Top level parameters
  if (config["camber_scaling_factor"]) {
    camber_scaling_factor = config["camber_scaling_factor"].as<double>();
  }
  if (config["effective_tire_r"]) {
    effective_tire_r = config["effective_tire_r"].as<double>();
  }
  if (config["d_bright"]) {
    d_bright = config["d_bright"].as<double>();
  }
  if (config["d_bleft"]) {
    d_bleft = config["d_bleft"].as<double>();
  }
  if (config["d_fright"]) {
    d_fright = config["d_fright"].as<double>();
  }
  if (config["d_fleft"]) {
    d_fleft = config["d_fleft"].as<double>();
  }

  // Dimension
  if (config["dimension"]["UNLOADED_RADIUS"]) {
    UNLOADED_RADIUS = config["dimension"]["UNLOADED_RADIUS"].as<double>();
  }
  if (config["dimension"]["WIDTH"]) {
    WIDTH = config["dimension"]["WIDTH"].as<double>();
  }
  if (config["dimension"]["ASPECT_RATIO"]) {
    ASPECT_RATIO = config["dimension"]["ASPECT_RATIO"].as<double>();
  }
  if (config["dimension"]["RIM_RADIUS"]) {
    RIM_RADIUS = config["dimension"]["RIM_RADIUS"].as<double>();
  }
  if (config["dimension"]["RIM_WIDTH"]) {
    RIM_WIDTH = config["dimension"]["RIM_WIDTH"].as<double>();
  }

  // Operating Conditions
  if (config["operating_conditions"]["INFLPRES"]) {
    INFLPRES = config["operating_conditions"]["INFLPRES"].as<double>();
  }
  if (config["operating_conditions"]["NOMPRES"]) {
    NOMPRES = config["operating_conditions"]["NOMPRES"].as<double>();
  }

  // Inertia
  if (config["inertia"]["MASS"]) {
    MASS = config["inertia"]["MASS"].as<double>();
  }
  if (config["inertia"]["IXX"]) {
    IXX = config["inertia"]["IXX"].as<double>();
  }
  if (config["inertia"]["IYY"]) {
    IYY = config["inertia"]["IYY"].as<double>();
  }
  if (config["inertia"]["BELT_MASS"]) {
    BELT_MASS = config["inertia"]["BELT_MASS"].as<double>();
  }
  if (config["inertia"]["BELT_IXX"]) {
    BELT_IXX = config["inertia"]["BELT_IXX"].as<double>();
  }
  if (config["inertia"]["BELT_IYY"]) {
    BELT_IYY = config["inertia"]["BELT_IYY"].as<double>();
  }
  if (config["inertia"]["GRAVITY"]) {
    GRAVITY = config["inertia"]["GRAVITY"].as<double>();
  }

  wheel_inertia = 0.5 * MASS * (UNLOADED_RADIUS * UNLOADED_RADIUS);

  // Vertical
  if (config["vertical"]["FNOMIN"]) {
    FNOMIN = config["vertical"]["FNOMIN"].as<double>();
  }
  if (config["vertical"]["VERTICAL_STIFFNESS"]) {
    VERTICAL_STIFFNESS = config["vertical"]["VERTICAL_STIFFNESS"].as<double>();
  }
  if (config["vertical"]["VERTICAL_DAMPING"]) {
    VERTICAL_DAMPING = config["vertical"]["VERTICAL_DAMPING"].as<double>();
  }
  if (config["vertical"]["MC_CONTOUR_A"]) {
    MC_CONTOUR_A = config["vertical"]["MC_CONTOUR_A"].as<double>();
  }
  if (config["vertical"]["MC_CONTOUR_B"]) {
    MC_CONTOUR_B = config["vertical"]["MC_CONTOUR_B"].as<double>();
  }
  if (config["vertical"]["BREFF"]) {
    BREFF = config["vertical"]["BREFF"].as<double>();
  }
  if (config["vertical"]["DREFF"]) {
    DREFF = config["vertical"]["DREFF"].as<double>();
  }
  if (config["vertical"]["FREFF"]) {
    FREFF = config["vertical"]["FREFF"].as<double>();
  }
  if (config["vertical"]["Q_RE0"]) {
    Q_RE0 = config["vertical"]["Q_RE0"].as<double>();
  }
  if (config["vertical"]["Q_V1"]) {
    Q_V1 = config["vertical"]["Q_V1"].as<double>();
  }
  if (config["vertical"]["Q_V2"]) {
    Q_V2 = config["vertical"]["Q_V2"].as<double>();
  }
  if (config["vertical"]["Q_FZ2"]) {
    Q_FZ2 = config["vertical"]["Q_FZ2"].as<double>();
  }
  if (config["vertical"]["Q_FCX"]) {
    Q_FCX = config["vertical"]["Q_FCX"].as<double>();
  }
  if (config["vertical"]["Q_FCY"]) {
    Q_FCY = config["vertical"]["Q_FCY"].as<double>();
  }
  if (config["vertical"]["Q_CAM"]) {
    Q_CAM = config["vertical"]["Q_CAM"].as<double>();
  }
  if (config["vertical"]["PFZ1"]) {
    PFZ1 = config["vertical"]["PFZ1"].as<double>();
  }
  if (config["vertical"]["Q_FCY2"]) {
    Q_FCY2 = config["vertical"]["Q_FCY2"].as<double>();
  }
  if (config["vertical"]["Q_CAM1"]) {
    Q_CAM1 = config["vertical"]["Q_CAM1"].as<double>();
  }
  if (config["vertical"]["Q_CAM2"]) {
    Q_CAM2 = config["vertical"]["Q_CAM2"].as<double>();
  }
  if (config["vertical"]["Q_CAM3"]) {
    Q_CAM3 = config["vertical"]["Q_CAM3"].as<double>();
  }
  if (config["vertical"]["Q_FYS1"]) {
    Q_FYS1 = config["vertical"]["Q_FYS1"].as<double>();
  }
  if (config["vertical"]["Q_FYS2"]) {
    Q_FYS2 = config["vertical"]["Q_FYS2"].as<double>();
  }
  if (config["vertical"]["Q_FYS3"]) {
    Q_FYS3 = config["vertical"]["Q_FYS3"].as<double>();
  }
  if (config["vertical"]["BOTTOM_OFFST"]) {
    BOTTOM_OFFST = config["vertical"]["BOTTOM_OFFST"].as<double>();
  }
  if (config["vertical"]["BOTTOM_STIFF"]) {
    BOTTOM_STIFF = config["vertical"]["BOTTOM_STIFF"].as<double>();
  }

  // Structural
  if (config["structural"]["LONGITUDINAL_STIFFNESS"]) {
    LONGITUDINAL_STIFFNESS = config["structural"]["LONGITUDINAL_STIFFNESS"].as<double>();
  }
  if (config["structural"]["LATERAL_STIFFNESS"]) {
    LATERAL_STIFFNESS = config["structural"]["LATERAL_STIFFNESS"].as<double>();
  }
  if (config["structural"]["YAW_STIFFNESS"]) {
    YAW_STIFFNESS = config["structural"]["YAW_STIFFNESS"].as<double>();
  }
  if (config["structural"]["FREQ_LONG"]) {
    FREQ_LONG = config["structural"]["FREQ_LONG"].as<double>();
  }
  if (config["structural"]["FREQ_LAT"]) {
    FREQ_LAT = config["structural"]["FREQ_LAT"].as<double>();
  }
  if (config["structural"]["FREQ_YAW"]) {
    FREQ_YAW = config["structural"]["FREQ_YAW"].as<double>();
  }
  if (config["structural"]["FREQ_WINDUP"]) {
    FREQ_WINDUP = config["structural"]["FREQ_WINDUP"].as<double>();
  }
  if (config["structural"]["DAMP_LONG"]) {
    DAMP_LONG = config["structural"]["DAMP_LONG"].as<double>();
  }
  if (config["structural"]["DAMP_LAT"]) {
    DAMP_LAT = config["structural"]["DAMP_LAT"].as<double>();
  }
  if (config["structural"]["DAMP_YAW"]) {
    DAMP_YAW = config["structural"]["DAMP_YAW"].as<double>();
  }
  if (config["structural"]["DAMP_WINDUP"]) {
    DAMP_WINDUP = config["structural"]["DAMP_WINDUP"].as<double>();
  }
  if (config["structural"]["DAMP_RESIDUAL"]) {
    DAMP_RESIDUAL = config["structural"]["DAMP_RESIDUAL"].as<double>();
  }
  if (config["structural"]["DAMP_VLOW"]) {
    DAMP_VLOW = config["structural"]["DAMP_VLOW"].as<double>();
  }
  if (config["structural"]["Q_BVX"]) {
    Q_BVX = config["structural"]["Q_BVX"].as<double>();
  }
  if (config["structural"]["Q_BVT"]) {
    Q_BVT = config["structural"]["Q_BVT"].as<double>();
  }
  if (config["structural"]["PCFX1"]) {
    PCFX1 = config["structural"]["PCFX1"].as<double>();
  }
  if (config["structural"]["PCFX2"]) {
    PCFX2 = config["structural"]["PCFX2"].as<double>();
  }
  if (config["structural"]["PCFX3"]) {
    PCFX3 = config["structural"]["PCFX3"].as<double>();
  }
  if (config["structural"]["PCFY1"]) {
    PCFY1 = config["structural"]["PCFY1"].as<double>();
  }
  if (config["structural"]["PCFY2"]) {
    PCFY2 = config["structural"]["PCFY2"].as<double>();
  }
  if (config["structural"]["PCFY3"]) {
    PCFY3 = config["structural"]["PCFY3"].as<double>();
  }
  if (config["structural"]["PCMZ1"]) {
    PCMZ1 = config["structural"]["PCMZ1"].as<double>();
  }

  // Contact patch
  if (config["contact_patch"]["Q_RA1"]) {
    Q_RA1 = config["contact_patch"]["Q_RA1"].as<double>();
  }
  if (config["contact_patch"]["Q_RA2"]) {
    Q_RA2 = config["contact_patch"]["Q_RA2"].as<double>();
  }
  if (config["contact_patch"]["Q_RB1"]) {
    Q_RB1 = config["contact_patch"]["Q_RB1"].as<double>();
  }
  if (config["contact_patch"]["Q_RB2"]) {
    Q_RB2 = config["contact_patch"]["Q_RB2"].as<double>();
  }
  if (config["contact_patch"]["ELLIPS_SHIFT"]) {
    ELLIPS_SHIFT = config["contact_patch"]["ELLIPS_SHIFT"].as<double>();
  }
  if (config["contact_patch"]["ELLIPS_LENGTH"]) {
    ELLIPS_LENGTH = config["contact_patch"]["ELLIPS_LENGTH"].as<double>();
  }
  if (config["contact_patch"]["ELLIPS_HEIGHT"]) {
    ELLIPS_HEIGHT = config["contact_patch"]["ELLIPS_HEIGHT"].as<double>();
  }
  if (config["contact_patch"]["ELLIPS_ORDER"]) {
    ELLIPS_ORDER = config["contact_patch"]["ELLIPS_ORDER"].as<double>();
  }
  if (config["contact_patch"]["ELLIPS_MAX_STEP"]) {
    ELLIPS_MAX_STEP = config["contact_patch"]["ELLIPS_MAX_STEP"].as<double>();
  }
  if (config["contact_patch"]["ELLIPS_NWIDTH"]) {
    ELLIPS_NWIDTH = config["contact_patch"]["ELLIPS_NWIDTH"].as<double>();
  }
  if (config["contact_patch"]["ELLIPS_NLENGTH"]) {
    ELLIPS_NLENGTH = config["contact_patch"]["ELLIPS_NLENGTH"].as<double>();
  }

  // Inflation pressure range
  if (config["inflation_pressure_range"]["PRESMIN"]) {
    PRESMIN = config["inflation_pressure_range"]["PRESMIN"].as<double>();
  }
  if (config["inflation_pressure_range"]["PRESMAX"]) {
    PRESMAX = config["inflation_pressure_range"]["PRESMAX"].as<double>();
  }

  // Vertical force range
  if (config["vertical_force_range"]["FZMIN"]) {
    FZMIN = config["vertical_force_range"]["FZMIN"].as<double>();
  }
  if (config["vertical_force_range"]["FZMAX"]) {
    FZMAX = config["vertical_force_range"]["FZMAX"].as<double>();
  }

  // Long slip range
  if (config["long_slip_range"]["KPUMIN"]) {
    KPUMIN = config["long_slip_range"]["KPUMIN"].as<double>();
  }
  if (config["long_slip_range"]["KPUMAX"]) {
    KPUMAX = config["long_slip_range"]["KPUMAX"].as<double>();
  }

  // Slip angle range
  if (config["slip_angle_range"]["ALPMIN"]) {
    ALPMIN = config["slip_angle_range"]["ALPMIN"].as<double>();
  }
  if (config["slip_angle_range"]["ALPMAX"]) {
    ALPMAX = config["slip_angle_range"]["ALPMAX"].as<double>();
  }

  // Inclination angle range
  if (config["inclination_angle_range"]["CAMMIN"]) {
    CAMMIN = config["inclination_angle_range"]["CAMMIN"].as<double>();
  }
  if (config["inclination_angle_range"]["CAMMAX"]) {
    CAMMAX = config["inclination_angle_range"]["CAMMAX"].as<double>();
  }

  // Scaling coefficients
  if (config["scaling_coefficients"]["LFZO"]) {
    LFZO = config["scaling_coefficients"]["LFZO"].as<double>();
  }
  if (config["scaling_coefficients"]["LCX"]) {
    LCX = config["scaling_coefficients"]["LCX"].as<double>();
  }
  if (config["scaling_coefficients"]["LMUX"]) {
    LMUX = config["scaling_coefficients"]["LMUX"].as<double>();
  }
  if (config["scaling_coefficients"]["LEX"]) {
    LEX = config["scaling_coefficients"]["LEX"].as<double>();
  }
  if (config["scaling_coefficients"]["LKX"]) {
    LKX = config["scaling_coefficients"]["LKX"].as<double>();
  }
  if (config["scaling_coefficients"]["LHX"]) {
    LHX = config["scaling_coefficients"]["LHX"].as<double>();
  }
  if (config["scaling_coefficients"]["LVX"]) {
    LVX = config["scaling_coefficients"]["LVX"].as<double>();
  }
  if (config["scaling_coefficients"]["LCY"]) {
    LCY = config["scaling_coefficients"]["LCY"].as<double>();
  }
  if (config["scaling_coefficients"]["LMUY"]) {
    LMUY = config["scaling_coefficients"]["LMUY"].as<double>();
  }
  if (config["scaling_coefficients"]["LEY"]) {
    LEY = config["scaling_coefficients"]["LEY"].as<double>();
  }
  if (config["scaling_coefficients"]["LKY"]) {
    LKY = config["scaling_coefficients"]["LKY"].as<double>();
  }
  if (config["scaling_coefficients"]["LHY"]) {
    LHY = config["scaling_coefficients"]["LHY"].as<double>();
  }
  if (config["scaling_coefficients"]["LVY"]) {
    LVY = config["scaling_coefficients"]["LVY"].as<double>();
  }
  if (config["scaling_coefficients"]["LTR"]) {
    LTR = config["scaling_coefficients"]["LTR"].as<double>();
  }
  if (config["scaling_coefficients"]["LRES"]) {
    LRES = config["scaling_coefficients"]["LRES"].as<double>();
  }
  if (config["scaling_coefficients"]["LXAL"]) {
    LXAL = config["scaling_coefficients"]["LXAL"].as<double>();
  }
  if (config["scaling_coefficients"]["LYKA"]) {
    LYKA = config["scaling_coefficients"]["LYKA"].as<double>();
  }
  if (config["scaling_coefficients"]["LVYKA"]) {
    LVYKA = config["scaling_coefficients"]["LVYKA"].as<double>();
  }
  if (config["scaling_coefficients"]["LS"]) {
    LS = config["scaling_coefficients"]["LS"].as<double>();
  }
  if (config["scaling_coefficients"]["LKYC"]) {
    LKYC = config["scaling_coefficients"]["LKYC"].as<double>();
  }
  if (config["scaling_coefficients"]["LKZC"]) {
    LKZC = config["scaling_coefficients"]["LKZC"].as<double>();
  }
  if (config["scaling_coefficients"]["LVMX"]) {
    LVMX = config["scaling_coefficients"]["LVMX"].as<double>();
  }
  if (config["scaling_coefficients"]["LMX"]) {
    LMX = config["scaling_coefficients"]["LMX"].as<double>();
  }
  if (config["scaling_coefficients"]["LMY"]) {
    LMY = config["scaling_coefficients"]["LMY"].as<double>();
  }
  if (config["scaling_coefficients"]["LMP"]) {
    LMP = config["scaling_coefficients"]["LMP"].as<double>();
  }

  // Longitudinal coefficients
  if (config["longitudinal_coefficients"]["PCX1"]) {
    PCX1 = config["longitudinal_coefficients"]["PCX1"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PDX1"]) {
    PDX1 = config["longitudinal_coefficients"]["PDX1"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PDX2"]) {
    PDX2 = config["longitudinal_coefficients"]["PDX2"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PDX3"]) {
    PDX3 = config["longitudinal_coefficients"]["PDX3"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PEX1"]) {
    PEX1 = config["longitudinal_coefficients"]["PEX1"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PEX2"]) {
    PEX2 = config["longitudinal_coefficients"]["PEX2"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PEX3"]) {
    PEX3 = config["longitudinal_coefficients"]["PEX3"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PEX4"]) {
    PEX4 = config["longitudinal_coefficients"]["PEX4"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PKX1"]) {
    PKX1 = config["longitudinal_coefficients"]["PKX1"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PKX2"]) {
    PKX2 = config["longitudinal_coefficients"]["PKX2"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PKX3"]) {
    PKX3 = config["longitudinal_coefficients"]["PKX3"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PHX1"]) {
    PHX1 = config["longitudinal_coefficients"]["PHX1"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PHX2"]) {
    PHX2 = config["longitudinal_coefficients"]["PHX2"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PVX1"]) {
    PVX1 = config["longitudinal_coefficients"]["PVX1"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PVX2"]) {
    PVX2 = config["longitudinal_coefficients"]["PVX2"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PPX1"]) {
    PPX1 = config["longitudinal_coefficients"]["PPX1"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PPX2"]) {
    PPX2 = config["longitudinal_coefficients"]["PPX2"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PPX3"]) {
    PPX3 = config["longitudinal_coefficients"]["PPX3"].as<double>();
  }
  if (config["longitudinal_coefficients"]["PPX4"]) {
    PPX4 = config["longitudinal_coefficients"]["PPX4"].as<double>();
  }
  if (config["longitudinal_coefficients"]["RBX1"]) {
    RBX1 = config["longitudinal_coefficients"]["RBX1"].as<double>();
  }
  if (config["longitudinal_coefficients"]["RBX2"]) {
    RBX2 = config["longitudinal_coefficients"]["RBX2"].as<double>();
  }
  if (config["longitudinal_coefficients"]["RBX3"]) {
    RBX3 = config["longitudinal_coefficients"]["RBX3"].as<double>();
  }
  if (config["longitudinal_coefficients"]["RCX1"]) {
    RCX1 = config["longitudinal_coefficients"]["RCX1"].as<double>();
  }
  if (config["longitudinal_coefficients"]["REX1"]) {
    REX1 = config["longitudinal_coefficients"]["REX1"].as<double>();
  }
  if (config["longitudinal_coefficients"]["REX2"]) {
    REX2 = config["longitudinal_coefficients"]["REX2"].as<double>();
  }
  if (config["longitudinal_coefficients"]["RHX1"]) {
    RHX1 = config["longitudinal_coefficients"]["RHX1"].as<double>();
  }

  // Overturning coefficients
  if (config["overturning_coefficients"]["QSX1"]) {
    QSX1 = config["overturning_coefficients"]["QSX1"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX2"]) {
    QSX2 = config["overturning_coefficients"]["QSX2"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX3"]) {
    QSX3 = config["overturning_coefficients"]["QSX3"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX4"]) {
    QSX4 = config["overturning_coefficients"]["QSX4"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX5"]) {
    QSX5 = config["overturning_coefficients"]["QSX5"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX6"]) {
    QSX6 = config["overturning_coefficients"]["QSX6"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX7"]) {
    QSX7 = config["overturning_coefficients"]["QSX7"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX8"]) {
    QSX8 = config["overturning_coefficients"]["QSX8"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX9"]) {
    QSX9 = config["overturning_coefficients"]["QSX9"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX10"]) {
    QSX10 = config["overturning_coefficients"]["QSX10"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX11"]) {
    QSX11 = config["overturning_coefficients"]["QSX11"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX12"]) {
    QSX12 = config["overturning_coefficients"]["QSX12"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX13"]) {
    QSX13 = config["overturning_coefficients"]["QSX13"].as<double>();
  }
  if (config["overturning_coefficients"]["QSX14"]) {
    QSX14 = config["overturning_coefficients"]["QSX14"].as<double>();
  }
  if (config["overturning_coefficients"]["PPMX1"]) {
    PPMX1 = config["overturning_coefficients"]["PPMX1"].as<double>();
  }

  // Lateral coefficients
  if (config["lateral_coefficients"]["PCY1"]) {
    PCY1 = config["lateral_coefficients"]["PCY1"].as<double>();
  }
  if (config["lateral_coefficients"]["PDY1"]) {
    PDY1 = config["lateral_coefficients"]["PDY1"].as<double>();
  }
  if (config["lateral_coefficients"]["PDY2"]) {
    PDY2 = config["lateral_coefficients"]["PDY2"].as<double>();
  }
  if (config["lateral_coefficients"]["PDY3"]) {
    PDY3 = config["lateral_coefficients"]["PDY3"].as<double>();
  }
  if (config["lateral_coefficients"]["PEY1"]) {
    PEY1 = config["lateral_coefficients"]["PEY1"].as<double>();
  }
  if (config["lateral_coefficients"]["PEY2"]) {
    PEY2 = config["lateral_coefficients"]["PEY2"].as<double>();
  }
  if (config["lateral_coefficients"]["PEY3"]) {
    PEY3 = config["lateral_coefficients"]["PEY3"].as<double>();
  }
  if (config["lateral_coefficients"]["PEY4"]) {
    PEY4 = config["lateral_coefficients"]["PEY4"].as<double>();
  }
  if (config["lateral_coefficients"]["PEY5"]) {
    PEY5 = config["lateral_coefficients"]["PEY5"].as<double>();
  }
  if (config["lateral_coefficients"]["PKY1"]) {
    PKY1 = config["lateral_coefficients"]["PKY1"].as<double>();
  }
  if (config["lateral_coefficients"]["PKY2"]) {
    PKY2 = config["lateral_coefficients"]["PKY2"].as<double>();
  }
  if (config["lateral_coefficients"]["PKY3"]) {
    PKY3 = config["lateral_coefficients"]["PKY3"].as<double>();
  }
  if (config["lateral_coefficients"]["PKY4"]) {
    PKY4 = config["lateral_coefficients"]["PKY4"].as<double>();
  }
  if (config["lateral_coefficients"]["PKY5"]) {
    PKY5 = config["lateral_coefficients"]["PKY5"].as<double>();
  }
  if (config["lateral_coefficients"]["PKY6"]) {
    PKY6 = config["lateral_coefficients"]["PKY6"].as<double>();
  }
  if (config["lateral_coefficients"]["PKY7"]) {
    PKY7 = config["lateral_coefficients"]["PKY7"].as<double>();
  }
  if (config["lateral_coefficients"]["PHY1"]) {
    PHY1 = config["lateral_coefficients"]["PHY1"].as<double>();
  }
  if (config["lateral_coefficients"]["PHY2"]) {
    PHY2 = config["lateral_coefficients"]["PHY2"].as<double>();
  }
  if (config["lateral_coefficients"]["PVY1"]) {
    PVY1 = config["lateral_coefficients"]["PVY1"].as<double>();
  }
  if (config["lateral_coefficients"]["PVY2"]) {
    PVY2 = config["lateral_coefficients"]["PVY2"].as<double>();
  }
  if (config["lateral_coefficients"]["PVY3"]) {
    PVY3 = config["lateral_coefficients"]["PVY3"].as<double>();
  }
  if (config["lateral_coefficients"]["PVY4"]) {
    PVY4 = config["lateral_coefficients"]["PVY4"].as<double>();
  }
  if (config["lateral_coefficients"]["PPY1"]) {
    PPY1 = config["lateral_coefficients"]["PPY1"].as<double>();
  }
  if (config["lateral_coefficients"]["PPY2"]) {
    PPY2 = config["lateral_coefficients"]["PPY2"].as<double>();
  }
  if (config["lateral_coefficients"]["PPY3"]) {
    PPY3 = config["lateral_coefficients"]["PPY3"].as<double>();
  }
  if (config["lateral_coefficients"]["PPY4"]) {
    PPY4 = config["lateral_coefficients"]["PPY4"].as<double>();
  }
  if (config["lateral_coefficients"]["PPY5"]) {
    PPY5 = config["lateral_coefficients"]["PPY5"].as<double>();
  }
  if (config["lateral_coefficients"]["RBY1"]) {
    RBY1 = config["lateral_coefficients"]["RBY1"].as<double>();
  }
  if (config["lateral_coefficients"]["RBY2"]) {
    RBY2 = config["lateral_coefficients"]["RBY2"].as<double>();
  }
  if (config["lateral_coefficients"]["RBY3"]) {
    RBY3 = config["lateral_coefficients"]["RBY3"].as<double>();
  }
  if (config["lateral_coefficients"]["RBY4"]) {
    RBY4 = config["lateral_coefficients"]["RBY4"].as<double>();
  }
  if (config["lateral_coefficients"]["RCY1"]) {
    RCY1 = config["lateral_coefficients"]["RCY1"].as<double>();
  }
  if (config["lateral_coefficients"]["REY1"]) {
    REY1 = config["lateral_coefficients"]["REY1"].as<double>();
  }
  if (config["lateral_coefficients"]["REY2"]) {
    REY2 = config["lateral_coefficients"]["REY2"].as<double>();
  }
  if (config["lateral_coefficients"]["RHY1"]) {
    RHY1 = config["lateral_coefficients"]["RHY1"].as<double>();
  }
  if (config["lateral_coefficients"]["RHY2"]) {
    RHY2 = config["lateral_coefficients"]["RHY2"].as<double>();
  }
  if (config["lateral_coefficients"]["RVY1"]) {
    RVY1 = config["lateral_coefficients"]["RVY1"].as<double>();
  }
  if (config["lateral_coefficients"]["RVY2"]) {
    RVY2 = config["lateral_coefficients"]["RVY2"].as<double>();
  }
  if (config["lateral_coefficients"]["RVY3"]) {
    RVY3 = config["lateral_coefficients"]["RVY3"].as<double>();
  }
  if (config["lateral_coefficients"]["RVY4"]) {
    RVY4 = config["lateral_coefficients"]["RVY4"].as<double>();
  }
  if (config["lateral_coefficients"]["RVY5"]) {
    RVY5 = config["lateral_coefficients"]["RVY5"].as<double>();
  }
  if (config["lateral_coefficients"]["RVY6"]) {
    RVY6 = config["lateral_coefficients"]["RVY6"].as<double>();
  }

  // Rolling coefficients
  if (config["rolling_coefficients"]["QSY1"]) {
    QSY1 = config["rolling_coefficients"]["QSY1"].as<double>();
  }
  if (config["rolling_coefficients"]["QSY2"]) {
    QSY2 = config["rolling_coefficients"]["QSY2"].as<double>();
  }
  if (config["rolling_coefficients"]["QSY3"]) {
    QSY3 = config["rolling_coefficients"]["QSY3"].as<double>();
  }
  if (config["rolling_coefficients"]["QSY4"]) {
    QSY4 = config["rolling_coefficients"]["QSY4"].as<double>();
  }
  if (config["rolling_coefficients"]["QSY5"]) {
    QSY5 = config["rolling_coefficients"]["QSY5"].as<double>();
  }
  if (config["rolling_coefficients"]["QSY6"]) {
    QSY6 = config["rolling_coefficients"]["QSY6"].as<double>();
  }
  if (config["rolling_coefficients"]["QSY7"]) {
    QSY7 = config["rolling_coefficients"]["QSY7"].as<double>();
  }
  if (config["rolling_coefficients"]["QSY8"]) {
    QSY8 = config["rolling_coefficients"]["QSY8"].as<double>();
  }

  // Aligning coefficients
  if (config["aligning_coefficients"]["QBZ1"]) {
    QBZ1 = config["aligning_coefficients"]["QBZ1"].as<double>();
  }
  if (config["aligning_coefficients"]["QBZ2"]) {
    QBZ2 = config["aligning_coefficients"]["QBZ2"].as<double>();
  }
  if (config["aligning_coefficients"]["QBZ3"]) {
    QBZ3 = config["aligning_coefficients"]["QBZ3"].as<double>();
  }
  if (config["aligning_coefficients"]["QBZ4"]) {
    QBZ4 = config["aligning_coefficients"]["QBZ4"].as<double>();
  }
  if (config["aligning_coefficients"]["QBZ5"]) {
    QBZ5 = config["aligning_coefficients"]["QBZ5"].as<double>();
  }
  if (config["aligning_coefficients"]["QBZ9"]) {
    QBZ9 = config["aligning_coefficients"]["QBZ9"].as<double>();
  }
  if (config["aligning_coefficients"]["QBZ10"]) {
    QBZ10 = config["aligning_coefficients"]["QBZ10"].as<double>();
  }
  if (config["aligning_coefficients"]["QCZ1"]) {
    QCZ1 = config["aligning_coefficients"]["QCZ1"].as<double>();
  }
  if (config["aligning_coefficients"]["QDZ1"]) {
    QDZ1 = config["aligning_coefficients"]["QDZ1"].as<double>();
  }
  if (config["aligning_coefficients"]["QDZ2"]) {
    QDZ2 = config["aligning_coefficients"]["QDZ2"].as<double>();
  }
  if (config["aligning_coefficients"]["QDZ3"]) {
    QDZ3 = config["aligning_coefficients"]["QDZ3"].as<double>();
  }
  if (config["aligning_coefficients"]["QDZ4"]) {
    QDZ4 = config["aligning_coefficients"]["QDZ4"].as<double>();
  }
  if (config["aligning_coefficients"]["QDZ6"]) {
    QDZ6 = config["aligning_coefficients"]["QDZ6"].as<double>();
  }
  if (config["aligning_coefficients"]["QDZ7"]) {
    QDZ7 = config["aligning_coefficients"]["QDZ7"].as<double>();
  }
  if (config["aligning_coefficients"]["QDZ8"]) {
    QDZ8 = config["aligning_coefficients"]["QDZ8"].as<double>();
  }
  if (config["aligning_coefficients"]["QDZ9"]) {
    QDZ9 = config["aligning_coefficients"]["QDZ9"].as<double>();
  }
  if (config["aligning_coefficients"]["QDZ10"]) {
    QDZ10 = config["aligning_coefficients"]["QDZ10"].as<double>();
  }
  if (config["aligning_coefficients"]["QDZ11"]) {
    QDZ11 = config["aligning_coefficients"]["QDZ11"].as<double>();
  }
  if (config["aligning_coefficients"]["QEZ1"]) {
    QEZ1 = config["aligning_coefficients"]["QEZ1"].as<double>();
  }
  if (config["aligning_coefficients"]["QEZ2"]) {
    QEZ2 = config["aligning_coefficients"]["QEZ2"].as<double>();
  }
  if (config["aligning_coefficients"]["QEZ3"]) {
    QEZ3 = config["aligning_coefficients"]["QEZ3"].as<double>();
  }
  if (config["aligning_coefficients"]["QEZ4"]) {
    QEZ4 = config["aligning_coefficients"]["QEZ4"].as<double>();
  }
  if (config["aligning_coefficients"]["QEZ5"]) {
    QEZ5 = config["aligning_coefficients"]["QEZ5"].as<double>();
  }
  if (config["aligning_coefficients"]["QHZ1"]) {
    QHZ1 = config["aligning_coefficients"]["QHZ1"].as<double>();
  }
  if (config["aligning_coefficients"]["QHZ2"]) {
    QHZ2 = config["aligning_coefficients"]["QHZ2"].as<double>();
  }
  if (config["aligning_coefficients"]["QHZ3"]) {
    QHZ3 = config["aligning_coefficients"]["QHZ3"].as<double>();
  }
  if (config["aligning_coefficients"]["QHZ4"]) {
    QHZ4 = config["aligning_coefficients"]["QHZ4"].as<double>();
  }
  if (config["aligning_coefficients"]["PPZ1"]) {
    PPZ1 = config["aligning_coefficients"]["PPZ1"].as<double>();
  }
  if (config["aligning_coefficients"]["PPZ2"]) {
    PPZ2 = config["aligning_coefficients"]["PPZ2"].as<double>();
  }
  if (config["aligning_coefficients"]["SSZ1"]) {
    SSZ1 = config["aligning_coefficients"]["SSZ1"].as<double>();
  }
  if (config["aligning_coefficients"]["SSZ2"]) {
    SSZ2 = config["aligning_coefficients"]["SSZ2"].as<double>();
  }
  if (config["aligning_coefficients"]["SSZ3"]) {
    SSZ3 = config["aligning_coefficients"]["SSZ3"].as<double>();
  }
  if (config["aligning_coefficients"]["SSZ4"]) {
    SSZ4 = config["aligning_coefficients"]["SSZ4"].as<double>();
  }

  // Turnslip coefficients
  if (config["turnslip_coefficients"]["PECP1"]) {
    PECP1 = config["turnslip_coefficients"]["PECP1"].as<double>();
  }
  if (config["turnslip_coefficients"]["PECP2"]) {
    PECP2 = config["turnslip_coefficients"]["PECP2"].as<double>();
  }
  if (config["turnslip_coefficients"]["PDXP1"]) {
    PDXP1 = config["turnslip_coefficients"]["PDXP1"].as<double>();
  }
  if (config["turnslip_coefficients"]["PDXP2"]) {
    PDXP2 = config["turnslip_coefficients"]["PDXP2"].as<double>();
  }
  if (config["turnslip_coefficients"]["PDXP3"]) {
    PDXP3 = config["turnslip_coefficients"]["PDXP3"].as<double>();
  }
  if (config["turnslip_coefficients"]["PDXP4"]) {
    PDXP4 = config["turnslip_coefficients"]["PDXP4"].as<double>();
  }
  if (config["turnslip_coefficients"]["PDYP1"]) {
    PDYP1 = config["turnslip_coefficients"]["PDYP1"].as<double>();
  }
  if (config["turnslip_coefficients"]["PDYP2"]) {
    PDYP2 = config["turnslip_coefficients"]["PDYP2"].as<double>();
  }
  if (config["turnslip_coefficients"]["PDYP3"]) {
    PDYP3 = config["turnslip_coefficients"]["PDYP3"].as<double>();
  }
  if (config["turnslip_coefficients"]["PDYP4"]) {
    PDYP4 = config["turnslip_coefficients"]["PDYP4"].as<double>();
  }
  if (config["turnslip_coefficients"]["PKYP1"]) {
    PKYP1 = config["turnslip_coefficients"]["PKYP1"].as<double>();
  }
  if (config["turnslip_coefficients"]["PHYP1"]) {
    PHYP1 = config["turnslip_coefficients"]["PHYP1"].as<double>();
  }
  if (config["turnslip_coefficients"]["PHYP2"]) {
    PHYP2 = config["turnslip_coefficients"]["PHYP2"].as<double>();
  }
  if (config["turnslip_coefficients"]["PHYP3"]) {
    PHYP3 = config["turnslip_coefficients"]["PHYP3"].as<double>();
  }
  if (config["turnslip_coefficients"]["PHYP4"]) {
    PHYP4 = config["turnslip_coefficients"]["PHYP4"].as<double>();
  }

  // Configurations
  if (config["configurations"]["Amu"]) {
    Amu = config["configurations"]["Amu"].as<double>();
  }

  // Per-wheel data (fr, fl, rr, rl)
  if (config["fr"]["toe"]) {
    fr_toe = config["fr"]["toe"].as<double>();
  }
  if (config["fr"]["camber"]) {
    fr_camber = config["fr"]["camber"].as<double>();
  }
  if (config["fl"]["toe"]) {
    fl_toe = config["fl"]["toe"].as<double>();
  }
  if (config["fl"]["camber"]) {
    fl_camber = config["fl"]["camber"].as<double>();
  }
  if (config["rr"]["toe"]) {
    rr_toe = config["rr"]["toe"].as<double>();
  }
  if (config["rr"]["camber"]) {
    rr_camber = config["rr"]["camber"].as<double>();
  }
  if (config["rl"]["toe"]) {
    rl_toe = config["rl"]["toe"].as<double>();
  }
  if (config["rl"]["camber"]) {
    rl_camber = config["rl"]["camber"].as<double>();
  }
}

}  // namespace common_lib::car_parameters