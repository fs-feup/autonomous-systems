#pragma once

#include <yaml-cpp/yaml.h>

#include "common_lib/config_load/config_load.hpp"

namespace common_lib::car_parameters {

struct TireParameters {
  // Original fields
  double tire_B_lateral;
  double tire_C_lateral;
  double tire_D_lateral;
  double tire_E_lateral;
  double tire_B_longitudinal;
  double tire_C_longitudinal;
  double tire_D_longitudinal;
  double tire_E_longitudinal;

  // Additional fields
  double camber_scaling_factor;
  double effective_tire_r;
  double distance_to_CG;

  // Distances
  double d_bright;
  double d_bleft;
  double d_fright;
  double d_fleft;

  // Camber
  double camber_angle;

  // Dimension
  double UNLOADED_RADIUS;
  double WIDTH;
  double ASPECT_RATIO;
  double RIM_RADIUS;
  double RIM_WIDTH;

  // Operating conditions
  double INFLPRES;
  double NOMPRES;

  // Inertia
  double MASS;
  double IXX;
  double IYY;
  double BELT_MASS;
  double BELT_IXX;
  double BELT_IYY;
  double GRAVITY;

  // Vertical
  double FNOMIN;
  double VERTICAL_STIFFNESS;
  double VERTICAL_DAMPING;
  double MC_CONTOUR_A;
  double MC_CONTOUR_B;
  double BREFF;
  double DREFF;
  double FREFF;
  double Q_RE0;
  double Q_V1;
  double Q_V2;
  double Q_FZ2;
  double Q_FCX;
  double Q_FCY;
  double Q_CAM;
  double PFZ1;
  double Q_FCY2;
  double Q_CAM1;
  double Q_CAM2;
  double Q_CAM3;
  double Q_FYS1;
  double Q_FYS2;
  double Q_FYS3;
  double BOTTOM_OFFST;
  double BOTTOM_STIFF;

  // Structural
  double LONGITUDINAL_STIFFNESS;
  double LATERAL_STIFFNESS;
  double YAW_STIFFNESS;
  double FREQ_LONG;
  double FREQ_LAT;
  double FREQ_YAW;
  double FREQ_WINDUP;
  double DAMP_LONG;
  double DAMP_LAT;
  double DAMP_YAW;
  double DAMP_WINDUP;
  double DAMP_RESIDUAL;
  double DAMP_VLOW;
  double Q_BVX;
  double Q_BVT;
  double PCFX1;
  double PCFX2;
  double PCFX3;
  double PCFY1;
  double PCFY2;
  double PCFY3;
  double PCMZ1;

  // Contact patch
  double Q_RA1;
  double Q_RA2;
  double Q_RB1;
  double Q_RB2;
  double ELLIPS_SHIFT;
  double ELLIPS_LENGTH;
  double ELLIPS_HEIGHT;
  double ELLIPS_ORDER;
  double ELLIPS_MAX_STEP;
  double ELLIPS_NWIDTH;
  double ELLIPS_NLENGTH;

  // Inflation pressure range
  double PRESMIN;
  double PRESMAX;

  // Vertical force range
  double FZMIN;
  double FZMAX;

  // Long slip range
  double KPUMIN;
  double KPUMAX;

  // Slip angle range
  double ALPMIN;
  double ALPMAX;

  // Inclination angle range
  double CAMMIN;
  double CAMMAX;

  // Scaling coefficients
  double LFZO;
  double LCX;
  double LMUX;
  double LEX;
  double LKX;
  double LHX;
  double LVX;
  double LCY;
  double LMUY;
  double LEY;
  double LKY;
  double LHY;
  double LVY;
  double LTR;
  double LRES;
  double LXAL;
  double LYKA;
  double LVYKA;
  double LS;
  double LKYC;
  double LKZC;
  double LVMX;
  double LMX;
  double LMY;
  double LMP;

  // Longitudinal coefficients
  double PCX1;
  double PDX1;
  double PDX2;
  double PDX3;
  double PEX1;
  double PEX2;
  double PEX3;
  double PEX4;
  double PKX1;
  double PKX2;
  double PKX3;
  double PHX1;
  double PHX2;
  double PVX1;
  double PVX2;
  double PPX1;
  double PPX2;
  double PPX3;
  double PPX4;
  double RBX1;
  double RBX2;
  double RBX3;
  double RCX1;
  double REX1;
  double REX2;
  double RHX1;

  // Overturning coefficients
  double QSX1;
  double QSX2;
  double QSX3;
  double QSX4;
  double QSX5;
  double QSX6;
  double QSX7;
  double QSX8;
  double QSX9;
  double QSX10;
  double QSX11;
  double QSX12;
  double QSX13;
  double QSX14;
  double PPMX1;

  // Lateral coefficients
  double PCY1;
  double PDY1;
  double PDY2;
  double PDY3;
  double PEY1;
  double PEY2;
  double PEY3;
  double PEY4;
  double PEY5;
  double PKY1;
  double PKY2;
  double PKY3;
  double PKY4;
  double PKY5;
  double PKY6;
  double PKY7;
  double PHY1;
  double PHY2;
  double PVY1;
  double PVY2;
  double PVY3;
  double PVY4;
  double PPY1;
  double PPY2;
  double PPY3;
  double PPY4;
  double PPY5;
  double RBY1;
  double RBY2;
  double RBY3;
  double RBY4;
  double RCY1;
  double REY1;
  double REY2;
  double RHY1;
  double RHY2;
  double RVY1;
  double RVY2;
  double RVY3;
  double RVY4;
  double RVY5;
  double RVY6;

  // Rolling coefficients
  double QSY1;
  double QSY2;
  double QSY3;
  double QSY4;
  double QSY5;
  double QSY6;
  double QSY7;
  double QSY8;

  // Aligning coefficients
  double QBZ1;
  double QBZ2;
  double QBZ3;
  double QBZ4;
  double QBZ5;
  double QBZ9;
  double QBZ10;
  double QCZ1;
  double QDZ1;
  double QDZ2;
  double QDZ3;
  double QDZ4;
  double QDZ6;
  double QDZ7;
  double QDZ8;
  double QDZ9;
  double QDZ10;
  double QDZ11;
  double QEZ1;
  double QEZ2;
  double QEZ3;
  double QEZ4;
  double QEZ5;
  double QHZ1;
  double QHZ2;
  double QHZ3;
  double QHZ4;
  double PPZ1;
  double PPZ2;
  double SSZ1;
  double SSZ2;
  double SSZ3;
  double SSZ4;

  // [TURNSLIP_COEFFICIENTS]
  double PECP1;
  double PECP2;
  double PDXP1;
  double PDXP2;
  double PDXP3;
  double PDXP4;
  double PDYP1;
  double PDYP2;
  double PDYP3;
  double PDYP4;
  double PKYP1;
  double PHYP1;
  double PHYP2;
  double PHYP3;
  double PHYP4;
  double QDTP1;
  double QBRP1;
  double QCRP1;
  double QCRP2;
  double QDRP1;
  double QDRP2;

  // [CONTACT_COEFFICIENTS]
  double PA1;
  double PA2;
  double PB1;
  double PB2;
  double PB3;
  double PAE;
  double PBE;
  double PCE;
  double PLS;
  double PW1;
  double PW2;
  double PW3;
  double PW4;
  double N_WIDTH;
  double N_WIDTH_HP;
  double N_LENGTH;
  double N_LENGTH_HP;
  double ROAD_SPACING;
  double ROAD_SPACING_HP;
  double MAX_HEIGHT;
  double CONTACT_THREADS;
  double CONTACT_THREADS_HP;

  // [DYNAMIC_COEFFICIENTS]
  double EPSNL;
  double MC;
  double IC;
  double KX;
  double KY;
  double KP;
  double CX;
  double CY;
  double CP;
  double EP;
  double EP12;
  double BF2;
  double BP1;
  double BP2;
  double BP3;
  double BP4;
  double CXZ1;
  double CXZ2;
  double CXP1;
  double CXX1;
  double CYZ1;
  double CYZ2;
  double CYP1;
  double CYY1;
  double CPZ1;

  // [BELT_PARAMETERS]
  double TYRE_MASS;
  double QMB;
  double QMC;
  double QIBY;
  double QIBXZ;
  double QIC;
  double QCBXZ;
  double QCBY;
  double QCBTH;
  double QCBGM;
  double QKBXZ;
  double QKBY;
  double QKBTH;
  double QKBGM;
  double QCCX;
  double QCCY;
  double QCCFI;
  double QKBX;
  double QKCY;
  double QKCFI;
  double QBVXZ;
  double QBVTH;

  // [LOADED_RADIUS_COEFFICIENTS]
  double QRE0;
  double QV1;
  double QV2;
  double QFCX1;
  double QFCY1;
  double QFCG1;
  double QFZ1;
  double QFZ2;
  double QFZ3;
  double QPFZ1;

  // Configurations
  double Amu;

  TireParameters(const std::string& config_path);
};

}  // namespace common_lib::car_parameters