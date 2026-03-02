#include "common_lib/car_parameters/tire_parameters.hpp"

namespace common_lib::car_parameters {

TireParameters::TireParameters(const std::string& config_name) {
  std::string config_path =
      common_lib::config_load::get_config_yaml_path("common_lib", "car/tire_model", config_name);
  YAML::Node config = YAML::LoadFile(config_path);
  config = config["tire"];
  if (config["tire_B_lateral"]) {
    tire_B_lateral = config["tire_B_lateral"].as<double>();
  }
  if (config["tire_C_lateral"]) {
    tire_C_lateral = config["tire_C_lateral"].as<double>();
  }
  if (config["tire_D_lateral"]) {
    tire_D_lateral = config["tire_D_lateral"].as<double>();
  }
  if (config["tire_E_lateral"]) {
    tire_E_lateral = config["tire_E_lateral"].as<double>();
  }
  if (config["tire_B_longitudinal"]) {
    tire_B_longitudinal = config["tire_B_longitudinal"].as<double>();
  }
  if (config["tire_C_longitudinal"]) {
    tire_C_longitudinal = config["tire_C_longitudinal"].as<double>();
  }
  if (config["tire_D_longitudinal"]) {
    tire_D_longitudinal = config["tire_D_longitudinal"].as<double>();
  }
  if (config["tire_E_longitudinal"]) {
    tire_E_longitudinal = config["tire_E_longitudinal"].as<double>();
  }
  if (config["camber_scaling_factor"]) {
    camber_scaling_factor = config["camber_scaling_factor"].as<double>();
  }
  if (config["effective_tire_r"]) {
    effective_tire_r = config["effective_tire_r"].as<double>();
  }
  if (config["distance_to_CG"]) {
    distance_to_CG = config["distance_to_CG"].as<double>();
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

  if (config["camber_angle"]) {
    camber_angle = config["camber_angle"].as<double>();
  }

  if (config["UNLOADED_RADIUS"]) {
    UNLOADED_RADIUS = config["UNLOADED_RADIUS"].as<double>();
  }
  if (config["WIDTH"]) {
    WIDTH = config["WIDTH"].as<double>();
  }
  if (config["ASPECT_RATIO"]) {
    ASPECT_RATIO = config["ASPECT_RATIO"].as<double>();
  }
  if (config["RIM_RADIUS"]) {
    RIM_RADIUS = config["RIM_RADIUS"].as<double>();
  }
  if (config["RIM_WIDTH"]) {
    RIM_WIDTH = config["RIM_WIDTH"].as<double>();
  }

  if (config["INFLPRES"]) {
    INFLPRES = config["INFLPRES"].as<double>();
  }
  if (config["NOMPRES"]) {
    NOMPRES = config["NOMPRES"].as<double>();
  }

  if (config["MASS"]) {
    MASS = config["MASS"].as<double>();
  }
  if (config["IXX"]) {
    IXX = config["IXX"].as<double>();
  }
  if (config["IYY"]) {
    IYY = config["IYY"].as<double>();
  }
  if (config["BELT_MASS"]) {
    BELT_MASS = config["BELT_MASS"].as<double>();
  }
  if (config["BELT_IXX"]) {
    BELT_IXX = config["BELT_IXX"].as<double>();
  }
  if (config["BELT_IYY"]) {
    BELT_IYY = config["BELT_IYY"].as<double>();
  }
  if (config["GRAVITY"]) {
    GRAVITY = config["GRAVITY"].as<double>();
  }

  if (config["FNOMIN"]) {
    FNOMIN = config["FNOMIN"].as<double>();
  }
  if (config["VERTICAL_STIFFNESS"]) {
    VERTICAL_STIFFNESS = config["VERTICAL_STIFFNESS"].as<double>();
  }
  if (config["VERTICAL_DAMPING"]) {
    VERTICAL_DAMPING = config["VERTICAL_DAMPING"].as<double>();
  }
  if (config["MC_CONTOUR_A"]) {
    MC_CONTOUR_A = config["MC_CONTOUR_A"].as<double>();
  }
  if (config["MC_CONTOUR_B"]) {
    MC_CONTOUR_B = config["MC_CONTOUR_B"].as<double>();
  }
  if (config["BREFF"]) {
    BREFF = config["BREFF"].as<double>();
  }
  if (config["DREFF"]) {
    DREFF = config["DREFF"].as<double>();
  }
  if (config["FREFF"]) {
    FREFF = config["FREFF"].as<double>();
  }
  if (config["Q_RE0"]) {
    Q_RE0 = config["Q_RE0"].as<double>();
  }
  if (config["Q_V1"]) {
    Q_V1 = config["Q_V1"].as<double>();
  }
  if (config["Q_V2"]) {
    Q_V2 = config["Q_V2"].as<double>();
  }
  if (config["Q_FZ2"]) {
    Q_FZ2 = config["Q_FZ2"].as<double>();
  }
  if (config["Q_FCX"]) {
    Q_FCX = config["Q_FCX"].as<double>();
  }
  if (config["Q_FCY"]) {
    Q_FCY = config["Q_FCY"].as<double>();
  }
  if (config["Q_CAM"]) {
    Q_CAM = config["Q_CAM"].as<double>();
  }
  if (config["PFZ1"]) {
    PFZ1 = config["PFZ1"].as<double>();
  }
  if (config["Q_FCY2"]) {
    Q_FCY2 = config["Q_FCY2"].as<double>();
  }
  if (config["Q_CAM1"]) {
    Q_CAM1 = config["Q_CAM1"].as<double>();
  }
  if (config["Q_CAM2"]) {
    Q_CAM2 = config["Q_CAM2"].as<double>();
  }
  if (config["Q_CAM3"]) {
    Q_CAM3 = config["Q_CAM3"].as<double>();
  }
  if (config["Q_FYS1"]) {
    Q_FYS1 = config["Q_FYS1"].as<double>();
  }
  if (config["Q_FYS2"]) {
    Q_FYS2 = config["Q_FYS2"].as<double>();
  }
  if (config["Q_FYS3"]) {
    Q_FYS3 = config["Q_FYS3"].as<double>();
  }
  if (config["BOTTOM_OFFST"]) {
    BOTTOM_OFFST = config["BOTTOM_OFFST"].as<double>();
  }
  if (config["BOTTOM_STIFF"]) {
    BOTTOM_STIFF = config["BOTTOM_STIFF"].as<double>();
  }

  // Structural
  if (config["LONGITUDINAL_STIFFNESS"]) {
    LONGITUDINAL_STIFFNESS = config["LONGITUDINAL_STIFFNESS"].as<double>();
  }
  if (config["LATERAL_STIFFNESS"]) {
    LATERAL_STIFFNESS = config["LATERAL_STIFFNESS"].as<double>();
  }
  if (config["YAW_STIFFNESS"]) {
    YAW_STIFFNESS = config["YAW_STIFFNESS"].as<double>();
  }
  if (config["FREQ_LONG"]) {
    FREQ_LONG = config["FREQ_LONG"].as<double>();
  }
  if (config["FREQ_LAT"]) {
    FREQ_LAT = config["FREQ_LAT"].as<double>();
  }
  if (config["FREQ_YAW"]) {
    FREQ_YAW = config["FREQ_YAW"].as<double>();
  }
  if (config["FREQ_WINDUP"]) {
    FREQ_WINDUP = config["FREQ_WINDUP"].as<double>();
  }
  if (config["DAMP_LONG"]) {
    DAMP_LONG = config["DAMP_LONG"].as<double>();
  }
  if (config["DAMP_LAT"]) {
    DAMP_LAT = config["DAMP_LAT"].as<double>();
  }
  if (config["DAMP_YAW"]) {
    DAMP_YAW = config["DAMP_YAW"].as<double>();
  }
  if (config["DAMP_WINDUP"]) {
    DAMP_WINDUP = config["DAMP_WINDUP"].as<double>();
  }
  if (config["DAMP_RESIDUAL"]) {
    DAMP_RESIDUAL = config["DAMP_RESIDUAL"].as<double>();
  }
  if (config["DAMP_VLOW"]) {
    DAMP_VLOW = config["DAMP_VLOW"].as<double>();
  }
  if (config["Q_BVX"]) {
    Q_BVX = config["Q_BVX"].as<double>();
  }
  if (config["Q_BVT"]) {
    Q_BVT = config["Q_BVT"].as<double>();
  }
  if (config["PCFX1"]) {
    PCFX1 = config["PCFX1"].as<double>();
  }
  if (config["PCFX2"]) {
    PCFX2 = config["PCFX2"].as<double>();
  }
  if (config["PCFX3"]) {
    PCFX3 = config["PCFX3"].as<double>();
  }
  if (config["PCFY1"]) {
    PCFY1 = config["PCFY1"].as<double>();
  }
  if (config["PCFY2"]) {
    PCFY2 = config["PCFY2"].as<double>();
  }
  if (config["PCFY3"]) {
    PCFY3 = config["PCFY3"].as<double>();
  }
  if (config["PCMZ1"]) {
    PCMZ1 = config["PCMZ1"].as<double>();
  }

  // Contact patch
  if (config["Q_RA1"]) {
    Q_RA1 = config["Q_RA1"].as<double>();
  }
  if (config["Q_RA2"]) {
    Q_RA2 = config["Q_RA2"].as<double>();
  }
  if (config["Q_RB1"]) {
    Q_RB1 = config["Q_RB1"].as<double>();
  }
  if (config["Q_RB2"]) {
    Q_RB2 = config["Q_RB2"].as<double>();
  }
  if (config["ELLIPS_SHIFT"]) {
    ELLIPS_SHIFT = config["ELLIPS_SHIFT"].as<double>();
  }
  if (config["ELLIPS_LENGTH"]) {
    ELLIPS_LENGTH = config["ELLIPS_LENGTH"].as<double>();
  }
  if (config["ELLIPS_HEIGHT"]) {
    ELLIPS_HEIGHT = config["ELLIPS_HEIGHT"].as<double>();
  }
  if (config["ELLIPS_ORDER"]) {
    ELLIPS_ORDER = config["ELLIPS_ORDER"].as<double>();
  }
  if (config["ELLIPS_MAX_STEP"]) {
    ELLIPS_MAX_STEP = config["ELLIPS_MAX_STEP"].as<double>();
  }
  if (config["ELLIPS_NWIDTH"]) {
    ELLIPS_NWIDTH = config["ELLIPS_NWIDTH"].as<double>();
  }
  if (config["ELLIPS_NLENGTH"]) {
    ELLIPS_NLENGTH = config["ELLIPS_NLENGTH"].as<double>();
  }

  // Inflation pressure range
  if (config["PRESMIN"]) {
    PRESMIN = config["PRESMIN"].as<double>();
  }
  if (config["PRESMAX"]) {
    PRESMAX = config["PRESMAX"].as<double>();
  }

  // Vertical force range
  if (config["FZMIN"]) {
    FZMIN = config["FZMIN"].as<double>();
  }
  if (config["FZMAX"]) {
    FZMAX = config["FZMAX"].as<double>();
  }

  // Long slip range
  if (config["KPUMIN"]) {
    KPUMIN = config["KPUMIN"].as<double>();
  }
  if (config["KPUMAX"]) {
    KPUMAX = config["KPUMAX"].as<double>();
  }

  // Slip angle range
  if (config["ALPMIN"]) {
    ALPMIN = config["ALPMIN"].as<double>();
  }
  if (config["ALPMAX"]) {
    ALPMAX = config["ALPMAX"].as<double>();
  }

  // Inclination angle range
  if (config["CAMMIN"]) {
    CAMMIN = config["CAMMIN"].as<double>();
  }
  if (config["CAMMAX"]) {
    CAMMAX = config["CAMMAX"].as<double>();
  }

  // Scaling coefficients
  if (config["LFZO"]) {
    LFZO = config["LFZO"].as<double>();
  }
  if (config["LCX"]) {
    LCX = config["LCX"].as<double>();
  }
  if (config["LMUX"]) {
    LMUX = config["LMUX"].as<double>();
  }
  if (config["LEX"]) {
    LEX = config["LEX"].as<double>();
  }
  if (config["LKX"]) {
    LKX = config["LKX"].as<double>();
  }
  if (config["LHX"]) {
    LHX = config["LHX"].as<double>();
  }
  if (config["LVX"]) {
    LVX = config["LVX"].as<double>();
  }
  if (config["LCY"]) {
    LCY = config["LCY"].as<double>();
  }
  if (config["LMUY"]) {
    LMUY = config["LMUY"].as<double>();
  }
  if (config["LEY"]) {
    LEY = config["LEY"].as<double>();
  }
  if (config["LKY"]) {
    LKY = config["LKY"].as<double>();
  }
  if (config["LHY"]) {
    LHY = config["LHY"].as<double>();
  }
  if (config["LVY"]) {
    LVY = config["LVY"].as<double>();
  }
  if (config["LTR"]) {
    LTR = config["LTR"].as<double>();
  }
  if (config["LRES"]) {
    LRES = config["LRES"].as<double>();
  }
  if (config["LXAL"]) {
    LXAL = config["LXAL"].as<double>();
  }
  if (config["LYKA"]) {
    LYKA = config["LYKA"].as<double>();
  }
  if (config["LVYKA"]) {
    LVYKA = config["LVYKA"].as<double>();
  }
  if (config["LS"]) {
    LS = config["LS"].as<double>();
  }
  if (config["LKYC"]) {
    LKYC = config["LKYC"].as<double>();
  }
  if (config["LKZC"]) {
    LKZC = config["LKZC"].as<double>();
  }
  if (config["LVMX"]) {
    LVMX = config["LVMX"].as<double>();
  }
  if (config["LMX"]) {
    LMX = config["LMX"].as<double>();
  }
  if (config["LMY"]) {
    LMY = config["LMY"].as<double>();
  }
  if (config["LMP"]) {
    LMP = config["LMP"].as<double>();
  }

  // Longitudinal coefficients
  if (config["PCX1"]) {
    PCX1 = config["PCX1"].as<double>();
  }
  if (config["PDX1"]) {
    PDX1 = config["PDX1"].as<double>();
  }
  if (config["PDX2"]) {
    PDX2 = config["PDX2"].as<double>();
  }
  if (config["PDX3"]) {
    PDX3 = config["PDX3"].as<double>();
  }
  if (config["PEX1"]) {
    PEX1 = config["PEX1"].as<double>();
  }
  if (config["PEX2"]) {
    PEX2 = config["PEX2"].as<double>();
  }
  if (config["PEX3"]) {
    PEX3 = config["PEX3"].as<double>();
  }
  if (config["PEX4"]) {
    PEX4 = config["PEX4"].as<double>();
  }
  if (config["PKX1"]) {
    PKX1 = config["PKX1"].as<double>();
  }
  if (config["PKX2"]) {
    PKX2 = config["PKX2"].as<double>();
  }
  if (config["PKX3"]) {
    PKX3 = config["PKX3"].as<double>();
  }
  if (config["PHX1"]) {
    PHX1 = config["PHX1"].as<double>();
  }
  if (config["PHX2"]) {
    PHX2 = config["PHX2"].as<double>();
  }
  if (config["PVX1"]) {
    PVX1 = config["PVX1"].as<double>();
  }
  if (config["PVX2"]) {
    PVX2 = config["PVX2"].as<double>();
  }
  if (config["PPX1"]) {
    PPX1 = config["PPX1"].as<double>();
  }
  if (config["PPX2"]) {
    PPX2 = config["PPX2"].as<double>();
  }
  if (config["PPX3"]) {
    PPX3 = config["PPX3"].as<double>();
  }
  if (config["PPX4"]) {
    PPX4 = config["PPX4"].as<double>();
  }
  if (config["RBX1"]) {
    RBX1 = config["RBX1"].as<double>();
  }
  if (config["RBX2"]) {
    RBX2 = config["RBX2"].as<double>();
  }
  if (config["RBX3"]) {
    RBX3 = config["RBX3"].as<double>();
  }
  if (config["RCX1"]) {
    RCX1 = config["RCX1"].as<double>();
  }
  if (config["REX1"]) {
    REX1 = config["REX1"].as<double>();
  }
  if (config["REX2"]) {
    REX2 = config["REX2"].as<double>();
  }
  if (config["RHX1"]) {
    RHX1 = config["RHX1"].as<double>();
  }

  // Overturning coefficients
  if (config["QSX1"]) {
    QSX1 = config["QSX1"].as<double>();
  }
  if (config["QSX2"]) {
    QSX2 = config["QSX2"].as<double>();
  }
  if (config["QSX3"]) {
    QSX3 = config["QSX3"].as<double>();
  }
  if (config["QSX4"]) {
    QSX4 = config["QSX4"].as<double>();
  }
  if (config["QSX5"]) {
    QSX5 = config["QSX5"].as<double>();
  }
  if (config["QSX6"]) {
    QSX6 = config["QSX6"].as<double>();
  }
  if (config["QSX7"]) {
    QSX7 = config["QSX7"].as<double>();
  }
  if (config["QSX8"]) {
    QSX8 = config["QSX8"].as<double>();
  }
  if (config["QSX9"]) {
    QSX9 = config["QSX9"].as<double>();
  }
  if (config["QSX10"]) {
    QSX10 = config["QSX10"].as<double>();
  }
  if (config["QSX11"]) {
    QSX11 = config["QSX11"].as<double>();
  }
  if (config["QSX12"]) {
    QSX12 = config["QSX12"].as<double>();
  }
  if (config["QSX13"]) {
    QSX13 = config["QSX13"].as<double>();
  }
  if (config["QSX14"]) {
    QSX14 = config["QSX14"].as<double>();
  }
  if (config["PPMX1"]) {
    PPMX1 = config["PPMX1"].as<double>();
  }

  // Lateral coefficients
  if (config["PCY1"]) {
    PCY1 = config["PCY1"].as<double>();
  }
  if (config["PDY1"]) {
    PDY1 = config["PDY1"].as<double>();
  }
  if (config["PDY2"]) {
    PDY2 = config["PDY2"].as<double>();
  }
  if (config["PDY3"]) {
    PDY3 = config["PDY3"].as<double>();
  }
  if (config["PEY1"]) {
    PEY1 = config["PEY1"].as<double>();
  }
  if (config["PEY2"]) {
    PEY2 = config["PEY2"].as<double>();
  }
  if (config["PEY3"]) {
    PEY3 = config["PEY3"].as<double>();
  }
  if (config["PEY4"]) {
    PEY4 = config["PEY4"].as<double>();
  }
  if (config["PEY5"]) {
    PEY5 = config["PEY5"].as<double>();
  }
  if (config["PKY1"]) {
    PKY1 = config["PKY1"].as<double>();
  }
  if (config["PKY2"]) {
    PKY2 = config["PKY2"].as<double>();
  }
  if (config["PKY3"]) {
    PKY3 = config["PKY3"].as<double>();
  }
  if (config["PKY4"]) {
    PKY4 = config["PKY4"].as<double>();
  }
  if (config["PKY5"]) {
    PKY5 = config["PKY5"].as<double>();
  }
  if (config["PKY6"]) {
    PKY6 = config["PKY6"].as<double>();
  }
  if (config["PKY7"]) {
    PKY7 = config["PKY7"].as<double>();
  }
  if (config["PHY1"]) {
    PHY1 = config["PHY1"].as<double>();
  }
  if (config["PHY2"]) {
    PHY2 = config["PHY2"].as<double>();
  }
  if (config["PVY1"]) {
    PVY1 = config["PVY1"].as<double>();
  }
  if (config["PVY2"]) {
    PVY2 = config["PVY2"].as<double>();
  }
  if (config["PVY3"]) {
    PVY3 = config["PVY3"].as<double>();
  }
  if (config["PVY4"]) {
    PVY4 = config["PVY4"].as<double>();
  }
  if (config["PPY1"]) {
    PPY1 = config["PPY1"].as<double>();
  }
  if (config["PPY2"]) {
    PPY2 = config["PPY2"].as<double>();
  }
  if (config["PPY3"]) {
    PPY3 = config["PPY3"].as<double>();
  }
  if (config["PPY4"]) {
    PPY4 = config["PPY4"].as<double>();
  }
  if (config["PPY5"]) {
    PPY5 = config["PPY5"].as<double>();
  }
  if (config["RBY1"]) {
    RBY1 = config["RBY1"].as<double>();
  }
  if (config["RBY2"]) {
    RBY2 = config["RBY2"].as<double>();
  }
  if (config["RBY3"]) {
    RBY3 = config["RBY3"].as<double>();
  }
  if (config["RBY4"]) {
    RBY4 = config["RBY4"].as<double>();
  }
  if (config["RCY1"]) {
    RCY1 = config["RCY1"].as<double>();
  }
  if (config["REY1"]) {
    REY1 = config["REY1"].as<double>();
  }
  if (config["REY2"]) {
    REY2 = config["REY2"].as<double>();
  }
  if (config["RHY1"]) {
    RHY1 = config["RHY1"].as<double>();
  }
  if (config["RHY2"]) {
    RHY2 = config["RHY2"].as<double>();
  }
  if (config["RVY1"]) {
    RVY1 = config["RVY1"].as<double>();
  }
  if (config["RVY2"]) {
    RVY2 = config["RVY2"].as<double>();
  }
  if (config["RVY3"]) {
    RVY3 = config["RVY3"].as<double>();
  }
  if (config["RVY4"]) {
    RVY4 = config["RVY4"].as<double>();
  }
  if (config["RVY5"]) {
    RVY5 = config["RVY5"].as<double>();
  }
  if (config["RVY6"]) {
    RVY6 = config["RVY6"].as<double>();
  }

  // Rolling coefficients
  if (config["QSY1"]) {
    QSY1 = config["QSY1"].as<double>();
  }
  if (config["QSY2"]) {
    QSY2 = config["QSY2"].as<double>();
  }
  if (config["QSY3"]) {
    QSY3 = config["QSY3"].as<double>();
  }
  if (config["QSY4"]) {
    QSY4 = config["QSY4"].as<double>();
  }
  if (config["QSY5"]) {
    QSY5 = config["QSY5"].as<double>();
  }
  if (config["QSY6"]) {
    QSY6 = config["QSY6"].as<double>();
  }
  if (config["QSY7"]) {
    QSY7 = config["QSY7"].as<double>();
  }
  if (config["QSY8"]) {
    QSY8 = config["QSY8"].as<double>();
  }

  // Aligning coefficients
  if (config["QBZ1"]) {
    QBZ1 = config["QBZ1"].as<double>();
  }
  if (config["QBZ2"]) {
    QBZ2 = config["QBZ2"].as<double>();
  }
  if (config["QBZ3"]) {
    QBZ3 = config["QBZ3"].as<double>();
  }
  if (config["QBZ4"]) {
    QBZ4 = config["QBZ4"].as<double>();
  }
  if (config["QBZ5"]) {
    QBZ5 = config["QBZ5"].as<double>();
  }
  if (config["QBZ9"]) {
    QBZ9 = config["QBZ9"].as<double>();
  }
  if (config["QBZ10"]) {
    QBZ10 = config["QBZ10"].as<double>();
  }
  if (config["QCZ1"]) {
    QCZ1 = config["QCZ1"].as<double>();
  }
  if (config["QDZ1"]) {
    QDZ1 = config["QDZ1"].as<double>();
  }
  if (config["QDZ2"]) {
    QDZ2 = config["QDZ2"].as<double>();
  }
  if (config["QDZ3"]) {
    QDZ3 = config["QDZ3"].as<double>();
  }
  if (config["QDZ4"]) {
    QDZ4 = config["QDZ4"].as<double>();
  }
  if (config["QDZ6"]) {
    QDZ6 = config["QDZ6"].as<double>();
  }
  if (config["QDZ7"]) {
    QDZ7 = config["QDZ7"].as<double>();
  }
  if (config["QDZ8"]) {
    QDZ8 = config["QDZ8"].as<double>();
  }
  if (config["QDZ9"]) {
    QDZ9 = config["QDZ9"].as<double>();
  }
  if (config["QDZ10"]) {
    QDZ10 = config["QDZ10"].as<double>();
  }
  if (config["QDZ11"]) {
    QDZ11 = config["QDZ11"].as<double>();
  }
  if (config["QEZ1"]) {
    QEZ1 = config["QEZ1"].as<double>();
  }
  if (config["QEZ2"]) {
    QEZ2 = config["QEZ2"].as<double>();
  }
  if (config["QEZ3"]) {
    QEZ3 = config["QEZ3"].as<double>();
  }
  if (config["QEZ4"]) {
    QEZ4 = config["QEZ4"].as<double>();
  }
  if (config["QEZ5"]) {
    QEZ5 = config["QEZ5"].as<double>();
  }
  if (config["QHZ1"]) {
    QHZ1 = config["QHZ1"].as<double>();
  }
  if (config["QHZ2"]) {
    QHZ2 = config["QHZ2"].as<double>();
  }
  if (config["QHZ3"]) {
    QHZ3 = config["QHZ3"].as<double>();
  }
  if (config["QHZ4"]) {
    QHZ4 = config["QHZ4"].as<double>();
  }
  if (config["PPZ1"]) {
    PPZ1 = config["PPZ1"].as<double>();
  }
  if (config["PPZ2"]) {
    PPZ2 = config["PPZ2"].as<double>();
  }
  if (config["SSZ1"]) {
    SSZ1 = config["SSZ1"].as<double>();
  }
  if (config["SSZ2"]) {
    SSZ2 = config["SSZ2"].as<double>();
  }
  if (config["SSZ3"]) {
    SSZ3 = config["SSZ3"].as<double>();
  }
  if (config["SSZ4"]) {
    SSZ4 = config["SSZ4"].as<double>();
  }

  // Turnslip coefficients
  if (config["PECP1"]) {
    PECP1 = config["PECP1"].as<double>();
  }
  if (config["PECP2"]) {
    PECP2 = config["PECP2"].as<double>();
  }
  if (config["PDXP1"]) {
    PDXP1 = config["PDXP1"].as<double>();
  }
  if (config["PDXP2"]) {
    PDXP2 = config["PDXP2"].as<double>();
  }
  if (config["PDXP3"]) {
    PDXP3 = config["PDXP3"].as<double>();
  }
  if (config["PDXP4"]) {
    PDXP4 = config["PDXP4"].as<double>();
  }
  if (config["PDYP1"]) {
    PDYP1 = config["PDYP1"].as<double>();
  }
  if (config["PDYP2"]) {
    PDYP2 = config["PDYP2"].as<double>();
  }
  if (config["PDYP3"]) {
    PDYP3 = config["PDYP3"].as<double>();
  }
  if (config["PDYP4"]) {
    PDYP4 = config["PDYP4"].as<double>();
  }
  if (config["PKYP1"]) {
    PKYP1 = config["PKYP1"].as<double>();
  }
  if (config["PHYP1"]) {
    PHYP1 = config["PHYP1"].as<double>();
  }
  if (config["PHYP2"]) {
    PHYP2 = config["PHYP2"].as<double>();
  }
  if (config["PHYP3"]) {
    PHYP3 = config["PHYP3"].as<double>();
  }
  if (config["PHYP4"]) {
    PHYP4 = config["PHYP4"].as<double>();
  }

  // Configurations
  if (config["Amu"]) {
    Amu = config["Amu"].as<double>();
  }
}
}  // namespace common_lib::car_parameters
