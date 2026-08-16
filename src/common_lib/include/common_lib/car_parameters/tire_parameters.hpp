#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

/*
 * Every member below carries a default. The loader assigns each field only when the key is
 * present in the tyre YAML (`if (config[...]) field = ...`), so any absent key previously left
 * the member reading uninitialised heap memory.
 *
 * Two of them are read by the MF6.2 model on every tyre evaluation: PPZ1 (absent from
 * 02_fitted_tire.yaml) and QDTP1 (never read by the loader at all). Both scale the pneumatic
 * trail, so the self-aligning moment -- and through it the yaw moment -- varied between runs of
 * the same scenario. Measured: 40 identically configured models produced 5 distinct yaw rates
 * after a single 1 ms step.
 *
 * The defaults chosen here are the values the simulator was in practice already using: the
 * garbage was consistently denormal, so `1 - PPZ1` evaluated to 1 and `cos(atan(QDTP1*Re*phi))`
 * to 1. Zero reproduces that exactly, and for QDTP1 it also matches the model's stated intent --
 * PacejkaMF6_2 already pins zeta7 and zeta8 to 1 to "ignore turnslip effects on self-aligning
 * moment", and zeta5 is the same effect on the trail. NOTE: the MF6.2 standard default for
 * QDTP1 is 1.0; enabling it is a deliberate plant change, not a bug fix.
 *
 * Scaling factors (L*) and Amu default to 1.0 because they are multiplicative; everything else
 * defaults to 0.0.
 */
struct TireParameters {
  // Original fields
  double tire_B_lateral = 0.0;
  double tire_C_lateral = 0.0;
  double tire_D_lateral = 0.0;
  double tire_E_lateral = 0.0;
  double tire_B_longitudinal = 0.0;
  double tire_C_longitudinal = 0.0;
  double tire_D_longitudinal = 0.0;
  double tire_E_longitudinal = 0.0;

  // Additional fields
  double camber_scaling_factor = 0.0;
  double effective_tire_r = 0.0;
  double fr_toe = 0.0;
  double fl_toe = 0.0;
  double rr_toe = 0.0;
  double rl_toe = 0.0;
  double wheel_inertia = 0.0;
  double slip_angle_relaxation_length = 0.0;
  double slip_ratio_relaxation_length = 0.0;

  // Distances
  double d_bright = 0.0;
  double d_bleft = 0.0;
  double d_fright = 0.0;
  double d_fleft = 0.0;

  // Camber
  double fr_camber = 0.0;
  double fl_camber = 0.0;
  double rr_camber = 0.0;
  double rl_camber = 0.0;

  // Dimension
  double UNLOADED_RADIUS = 0.0;
  double WIDTH = 0.0;
  double ASPECT_RATIO = 0.0;
  double RIM_RADIUS = 0.0;
  double RIM_WIDTH = 0.0;

  // Operating conditions
  double INFLPRES = 0.0;
  double NOMPRES = 0.0;

  // Inertia
  double MASS = 0.0;
  double IXX = 0.0;
  double IYY = 0.0;
  double BELT_MASS = 0.0;
  double BELT_IXX = 0.0;
  double BELT_IYY = 0.0;
  double GRAVITY = 0.0;

  // Vertical
  double FNOMIN = 0.0;
  double VERTICAL_STIFFNESS = 0.0;
  double VERTICAL_DAMPING = 0.0;
  double MC_CONTOUR_A = 0.0;
  double MC_CONTOUR_B = 0.0;
  double BREFF = 0.0;
  double DREFF = 0.0;
  double FREFF = 0.0;
  double Q_RE0 = 0.0;
  double Q_V1 = 0.0;
  double Q_V2 = 0.0;
  double Q_FZ2 = 0.0;
  double Q_FCX = 0.0;
  double Q_FCY = 0.0;
  double Q_CAM = 0.0;
  double PFZ1 = 0.0;
  double Q_FCY2 = 0.0;
  double Q_CAM1 = 0.0;
  double Q_CAM2 = 0.0;
  double Q_CAM3 = 0.0;
  double Q_FYS1 = 0.0;
  double Q_FYS2 = 0.0;
  double Q_FYS3 = 0.0;
  double BOTTOM_OFFST = 0.0;
  double BOTTOM_STIFF = 0.0;

  // Structural
  double LONGITUDINAL_STIFFNESS = 0.0;
  double LATERAL_STIFFNESS = 0.0;
  double YAW_STIFFNESS = 0.0;
  double FREQ_LONG = 0.0;
  double FREQ_LAT = 0.0;
  double FREQ_YAW = 0.0;
  double FREQ_WINDUP = 0.0;
  double DAMP_LONG = 0.0;
  double DAMP_LAT = 0.0;
  double DAMP_YAW = 0.0;
  double DAMP_WINDUP = 0.0;
  double DAMP_RESIDUAL = 0.0;
  double DAMP_VLOW = 0.0;
  double Q_BVX = 0.0;
  double Q_BVT = 0.0;
  double PCFX1 = 0.0;
  double PCFX2 = 0.0;
  double PCFX3 = 0.0;
  double PCFY1 = 0.0;
  double PCFY2 = 0.0;
  double PCFY3 = 0.0;
  double PCMZ1 = 0.0;

  // Contact patch
  double Q_RA1 = 0.0;
  double Q_RA2 = 0.0;
  double Q_RB1 = 0.0;
  double Q_RB2 = 0.0;
  double ELLIPS_SHIFT = 0.0;
  double ELLIPS_LENGTH = 0.0;
  double ELLIPS_HEIGHT = 0.0;
  double ELLIPS_ORDER = 0.0;
  double ELLIPS_MAX_STEP = 0.0;
  double ELLIPS_NWIDTH = 0.0;
  double ELLIPS_NLENGTH = 0.0;

  // Inflation pressure range
  double PRESMIN = 0.0;
  double PRESMAX = 0.0;

  // Vertical force range
  double FZMIN = 0.0;
  double FZMAX = 0.0;

  // Long slip range
  double KPUMIN = 0.0;
  double KPUMAX = 0.0;

  // Slip angle range
  double ALPMIN = 0.0;
  double ALPMAX = 0.0;

  // Inclination angle range
  double CAMMIN = 0.0;
  double CAMMAX = 0.0;

  // Scaling coefficients
  double LFZO = 1.0;
  double LCX = 1.0;
  double LMUX = 1.0;
  double LEX = 1.0;
  double LKX = 1.0;
  double LHX = 1.0;
  double LVX = 1.0;
  double LCY = 1.0;
  double LMUY = 1.0;
  double LEY = 1.0;
  double LKY = 1.0;
  double LHY = 1.0;
  double LVY = 1.0;
  double LTR = 1.0;
  double LRES = 1.0;
  double LXAL = 1.0;
  double LYKA = 1.0;
  double LVYKA = 1.0;
  double LS = 1.0;
  double LKYC = 1.0;
  double LKZC = 1.0;
  double LVMX = 1.0;
  double LMX = 1.0;
  double LMY = 1.0;
  double LMP = 1.0;

  // Longitudinal coefficients
  double PCX1 = 0.0;
  double PDX1 = 0.0;
  double PDX2 = 0.0;
  double PDX3 = 0.0;
  double PEX1 = 0.0;
  double PEX2 = 0.0;
  double PEX3 = 0.0;
  double PEX4 = 0.0;
  double PKX1 = 0.0;
  double PKX2 = 0.0;
  double PKX3 = 0.0;
  double PHX1 = 0.0;
  double PHX2 = 0.0;
  double PVX1 = 0.0;
  double PVX2 = 0.0;
  double PPX1 = 0.0;
  double PPX2 = 0.0;
  double PPX3 = 0.0;
  double PPX4 = 0.0;
  double RBX1 = 0.0;
  double RBX2 = 0.0;
  double RBX3 = 0.0;
  double RCX1 = 0.0;
  double REX1 = 0.0;
  double REX2 = 0.0;
  double RHX1 = 0.0;

  // Overturning coefficients
  double QSX1 = 0.0;
  double QSX2 = 0.0;
  double QSX3 = 0.0;
  double QSX4 = 0.0;
  double QSX5 = 0.0;
  double QSX6 = 0.0;
  double QSX7 = 0.0;
  double QSX8 = 0.0;
  double QSX9 = 0.0;
  double QSX10 = 0.0;
  double QSX11 = 0.0;
  double QSX12 = 0.0;
  double QSX13 = 0.0;
  double QSX14 = 0.0;
  double PPMX1 = 0.0;

  // Lateral coefficients
  double PCY1 = 0.0;
  double PDY1 = 0.0;
  double PDY2 = 0.0;
  double PDY3 = 0.0;
  double PEY1 = 0.0;
  double PEY2 = 0.0;
  double PEY3 = 0.0;
  double PEY4 = 0.0;
  double PEY5 = 0.0;
  double PKY1 = 0.0;
  double PKY2 = 0.0;
  double PKY3 = 0.0;
  double PKY4 = 0.0;
  double PKY5 = 0.0;
  double PKY6 = 0.0;
  double PKY7 = 0.0;
  double PHY1 = 0.0;
  double PHY2 = 0.0;
  double PVY1 = 0.0;
  double PVY2 = 0.0;
  double PVY3 = 0.0;
  double PVY4 = 0.0;
  double PPY1 = 0.0;
  double PPY2 = 0.0;
  double PPY3 = 0.0;
  double PPY4 = 0.0;
  double PPY5 = 0.0;
  double RBY1 = 0.0;
  double RBY2 = 0.0;
  double RBY3 = 0.0;
  double RBY4 = 0.0;
  double RCY1 = 0.0;
  double REY1 = 0.0;
  double REY2 = 0.0;
  double RHY1 = 0.0;
  double RHY2 = 0.0;
  double RVY1 = 0.0;
  double RVY2 = 0.0;
  double RVY3 = 0.0;
  double RVY4 = 0.0;
  double RVY5 = 0.0;
  double RVY6 = 0.0;

  // Rolling coefficients
  double QSY1 = 0.0;
  double QSY2 = 0.0;
  double QSY3 = 0.0;
  double QSY4 = 0.0;
  double QSY5 = 0.0;
  double QSY6 = 0.0;
  double QSY7 = 0.0;
  double QSY8 = 0.0;

  // Aligning coefficients
  double QBZ1 = 0.0;
  double QBZ2 = 0.0;
  double QBZ3 = 0.0;
  double QBZ4 = 0.0;
  double QBZ5 = 0.0;
  double QBZ9 = 0.0;
  double QBZ10 = 0.0;
  double QCZ1 = 0.0;
  double QDZ1 = 0.0;
  double QDZ2 = 0.0;
  double QDZ3 = 0.0;
  double QDZ4 = 0.0;
  double QDZ6 = 0.0;
  double QDZ7 = 0.0;
  double QDZ8 = 0.0;
  double QDZ9 = 0.0;
  double QDZ10 = 0.0;
  double QDZ11 = 0.0;
  double QEZ1 = 0.0;
  double QEZ2 = 0.0;
  double QEZ3 = 0.0;
  double QEZ4 = 0.0;
  double QEZ5 = 0.0;
  double QHZ1 = 0.0;
  double QHZ2 = 0.0;
  double QHZ3 = 0.0;
  double QHZ4 = 0.0;
  double PPZ1 = 0.0;
  double PPZ2 = 0.0;
  double SSZ1 = 0.0;
  double SSZ2 = 0.0;
  double SSZ3 = 0.0;
  double SSZ4 = 0.0;

  // [TURNSLIP_COEFFICIENTS]
  double PECP1 = 0.0;
  double PECP2 = 0.0;
  double PDXP1 = 0.0;
  double PDXP2 = 0.0;
  double PDXP3 = 0.0;
  double PDXP4 = 0.0;
  double PDYP1 = 0.0;
  double PDYP2 = 0.0;
  double PDYP3 = 0.0;
  double PDYP4 = 0.0;
  double PKYP1 = 0.0;
  double PHYP1 = 0.0;
  double PHYP2 = 0.0;
  double PHYP3 = 0.0;
  double PHYP4 = 0.0;
  double QDTP1 = 0.0;
  double QBRP1 = 0.0;
  double QCRP1 = 0.0;
  double QCRP2 = 0.0;
  double QDRP1 = 0.0;
  double QDRP2 = 0.0;

  // [CONTACT_COEFFICIENTS]
  double PA1 = 0.0;
  double PA2 = 0.0;
  double PB1 = 0.0;
  double PB2 = 0.0;
  double PB3 = 0.0;
  double PAE = 0.0;
  double PBE = 0.0;
  double PCE = 0.0;
  double PLS = 0.0;
  double PW1 = 0.0;
  double PW2 = 0.0;
  double PW3 = 0.0;
  double PW4 = 0.0;
  double N_WIDTH = 0.0;
  double N_WIDTH_HP = 0.0;
  double N_LENGTH = 0.0;
  double N_LENGTH_HP = 0.0;
  double ROAD_SPACING = 0.0;
  double ROAD_SPACING_HP = 0.0;
  double MAX_HEIGHT = 0.0;
  double CONTACT_THREADS = 0.0;
  double CONTACT_THREADS_HP = 0.0;

  // [DYNAMIC_COEFFICIENTS]
  double EPSNL = 0.0;
  double MC = 0.0;
  double IC = 0.0;
  double KX = 0.0;
  double KY = 0.0;
  double KP = 0.0;
  double CX = 0.0;
  double CY = 0.0;
  double CP = 0.0;
  double EP = 0.0;
  double EP12 = 0.0;
  double BF2 = 0.0;
  double BP1 = 0.0;
  double BP2 = 0.0;
  double BP3 = 0.0;
  double BP4 = 0.0;
  double CXZ1 = 0.0;
  double CXZ2 = 0.0;
  double CXP1 = 0.0;
  double CXX1 = 0.0;
  double CYZ1 = 0.0;
  double CYZ2 = 0.0;
  double CYP1 = 0.0;
  double CYY1 = 0.0;
  double CPZ1 = 0.0;

  // [BELT_PARAMETERS]
  double TYRE_MASS = 0.0;
  double QMB = 0.0;
  double QMC = 0.0;
  double QIBY = 0.0;
  double QIBXZ = 0.0;
  double QIC = 0.0;
  double QCBXZ = 0.0;
  double QCBY = 0.0;
  double QCBTH = 0.0;
  double QCBGM = 0.0;
  double QKBXZ = 0.0;
  double QKBY = 0.0;
  double QKBTH = 0.0;
  double QKBGM = 0.0;
  double QCCX = 0.0;
  double QCCY = 0.0;
  double QCCFI = 0.0;
  double QKBX = 0.0;
  double QKCY = 0.0;
  double QKCFI = 0.0;
  double QBVXZ = 0.0;
  double QBVTH = 0.0;

  // [LOADED_RADIUS_COEFFICIENTS]
  double QRE0 = 0.0;
  double QV1 = 0.0;
  double QV2 = 0.0;
  double QFCX1 = 0.0;
  double QFCY1 = 0.0;
  double QFCG1 = 0.0;
  double QFZ1 = 0.0;
  double QFZ2 = 0.0;
  double QFZ3 = 0.0;
  double QPFZ1 = 0.0;

  // Configurations
  double Amu = 1.0;

  TireParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters
