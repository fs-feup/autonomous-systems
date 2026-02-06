#pragma once

/*
    Helper structs to make code modular
  */
struct Velocity {
  double vx;
  double vy;
};
struct Force {
  double fx;
  double fy;
  double fz;
};

struct Wheels {
  float front_left;
  float front_right;
  float rear_left;
  float rear_right;
};

struct TireParameters {
  // ---------------- configuration values
  double Amu = 1.0;  // VD uses 1 but the pacejka book suggests 10
  // ---------------- dimension (geometry)
  double UNLOADED_RADIUS = 0.235;  // Free tire radius
  double WIDTH = 0.2159;           // Section width
  double ASPECT_RATIO = 0.55;      // Sidewall ratio
  double RIM_RADIUS = 0.1905;      // Rim radius
  double RIM_WIDTH = 0.152;        // Rim width

  // ---------------- operating conditions
  double INFLPRES = 220000;  // Actual inflation pressure
  double NOMPRES = 220000;   // Nominal reference pressure

  // ---------------- inertia
  double MASS = 17.451;    // Tire mass
  double IXX = 0.4;        // Rotational inertia (spin)
  double IYY = 0.7;        // Lateral inertia
  double BELT_MASS = 7;    // Belt mass
  double BELT_IXX = 0.34;  // Belt spin inertia
  double BELT_IYY = 0.6;   // Belt lateral inertia
  double GRAVITY = -9.81;  // Gravity sign convention

  // ---------------- vertical (Fz behavior)
  double FNOMIN = 1113.5001;           // Nominal load
  double VERTICAL_STIFFNESS = 200000;  // Radial stiffness
  double VERTICAL_DAMPING = 50;        // Radial damping
  double MC_CONTOUR_A = 0.5;           // Contact shape factor
  double MC_CONTOUR_B = 0.5;           // Contact shape factor
  double BREFF = 8.4;                  // Effective rolling radius B
  double DREFF = 0.27;                 // Effective rolling radius D
  double FREFF = 0.07;                 // Effective rolling radius F
  double Q_RE0 = 1;                    // Rolling radius scaling
  double Q_V1 = 0;                     // Speed effect on radius
  double Q_V2 = 0;                     // Speed² effect on radius
  double Q_FZ2 = 0.0001;               // Load² effect
  double Q_FCX = 0;                    // Fx influence on Fz
  double Q_FCY = 0;                    // Fy influence on Fz
  double Q_CAM = 0;                    // Camber influence on Fz
  double PFZ1 = 0.88241;               // Vertical force scaling
  double Q_FCY2 = -0.4751;             // Nonlinear Fy–Fz coupling
  double Q_CAM1 = 85.19;               // Camber stiffness term
  double Q_CAM2 = 257.4;               // Camber stiffness term
  double Q_CAM3 = 0.5119;              // Camber stiffness term
  double Q_FYS1 = -20496.4;            // Lateral deflection term
  double Q_FYS2 = -60000;              // Lateral deflection term
  double Q_FYS3 = 88211.7;             // Lateral deflection term
  double BOTTOM_OFFST = 0.01;          // Hard-stop offset
  double BOTTOM_STIFF = 2000000;       // Hard-stop stiffness

  // ---------------- structural (relaxation)
  double LONGITUDINAL_STIFFNESS = 300000;  // Longitudinal carcass stiffness
  double LATERAL_STIFFNESS = 100000;       // Lateral carcass stiffness
  double YAW_STIFFNESS = 5000;             // Torsional stiffness
  double FREQ_LONG = 80;                   // Longitudinal relaxation freq
  double FREQ_LAT = 40;                    // Lateral relaxation freq
  double FREQ_YAW = 50;                    // Yaw relaxation freq
  double FREQ_WINDUP = 70;                 // Wind-up relaxation freq
  double DAMP_LONG = 0.04;                 // Longitudinal damping
  double DAMP_LAT = 0.04;                  // Lateral damping
  double DAMP_YAW = 0.04;                  // Yaw damping
  double DAMP_WINDUP = 0.04;               // Wind-up damping
  double DAMP_RESIDUAL = 0.002;            // Residual damping
  double DAMP_VLOW = 0.001;                // Low-speed damping
  double Q_BVX = 0;                        // Fx–V coupling
  double Q_BVT = 0;                        // Fy–V coupling
  double PCFX1 = 0;                        // Fx structural shape
  double PCFX2 = 0;
  double PCFX3 = 0;
  double PCFY1 = 0;  // Fy structural shape
  double PCFY2 = 0;
  double PCFY3 = 0;
  double PCMZ1 = 0;  // Mz structural shape

  // ---------------- contact patch
  double Q_RA1 = 0.5;              // Patch length scaling
  double Q_RA2 = 1;                // Patch load effect
  double Q_RB1 = 1;                // Patch width scaling
  double Q_RB2 = -1;               // Patch width load effect
  double ELLIPS_SHIFT = 0.8;       // Ellipse shift
  double ELLIPS_LENGTH = 1;        // Ellipse length
  double ELLIPS_HEIGHT = 1;        // Ellipse height
  double ELLIPS_ORDER = 1.8;       // Ellipse shape order
  double ELLIPS_MAX_STEP = 0.025;  // Patch discretization
  double ELLIPS_NWIDTH = 10;       // Patch grid width
  double ELLIPS_NLENGTH = 10;      // Patch grid length

  // ---------------- ranges / limits
  double PRESMIN = 10000;    // Min pressure
  double PRESMAX = 1000000;  // Max pressure
  double FZMIN = 228.6;      // Min vertical load
  double FZMAX = 1600;       // Max vertical load
  double KPUMIN = -0.25;     // Min slip ratio
  double KPUMAX = 0.25;      // Max slip ratio
  double ALPMIN = -0.2094;   // Min slip angle
  double ALPMAX = 0.2094;    // Max slip angle
  double CAMMIN = -0.1047;   // Min camber
  double CAMMAX = 0.1047;    // Max camber

  // ---------------- scaling coefficients
  double LFZO = 1;   // Load scaling
  double LCX = 1;    // Fx shape scaling
  double LMUX = 1;   // Longitudinal friction scaling
  double LEX = 1;    // Fx curvature scaling
  double LKX = 1;    // Fx stiffness scaling
  double LHX = 0;    // Fx horizontal shift
  double LVX = 0;    // Fx vertical shift
  double LCY = 1;    // Fy shape scaling
  double LMUY = 1;   // Lateral friction scaling
  double LEY = 1;    // Fy curvature scaling
  double LKY = 1;    // Fy stiffness scaling
  double LHY = 0;    // Fy horizontal shift
  double LVY = 0;    // Fy vertical shift
  double LTR = 1;    // Trail scaling
  double LRES = 0;   // Residual moment scaling
  double LXAL = 1;   // Alpha scaling
  double LYKA = 1;   // Combined-slip scaling
  double LVYKA = 1;  // Combined-slip shift
  double LS = 1;     // Slip scaling
  double LKYC = 1;   // Combined Fy stiffness
  double LKZC = 1;   // Combined Mz stiffness
  double LVMX = 0;   // Fx moment shift
  double LMX = 1;    // Fx moment scaling
  double LMY = 1;    // Fy moment scaling
  double LMP = 1;    // Pressure scaling

  // ---------------- longitudinal MF (Fx)
  double PCX1 = 1.786;  // Fx shape factor
  double PDX1 = 2.933;  // Fx peak friction
  double PDX2 = -0.44;  // Fx load sensitivity
  double PDX3 = 20.8;   // Fx camber sensitivity
  double PEX1 = 0.871;  // Fx curvature
  double PEX2 = -0.038;
  double PEX3 = 0;
  double PEX4 = 0.071;
  double PKX1 = 85.31;   // Fx stiffness
  double PKX2 = -20.25;  // Fx stiffness load effect
  double PKX3 = 0.5;     // Fx stiffness camber effect
  double PHX1 = 0;       // Fx horizontal shift
  double PHX2 = 0;
  double PVX1 = 0;  // Fx vertical shift
  double PVX2 = 0;
  double PPX1 = 0;  // Fx pressure effect
  double PPX2 = 0;
  double PPX3 = 0;
  double PPX4 = 0;
  double RBX1 = 23.72;  // Combined-slip factor
  double RBX2 = 25.97;
  double RBX3 = 0;
  double RCX1 = 0.7495;
  double REX1 = -0.4759;
  double REX2 = 0.8109;
  double RHX1 = 0;

  // ---------------- lateral MF (Fy)
  double PCY1 = 1.6384;   // Fy shape factor
  double PDY1 = 2.3749;   // Fy peak friction
  double PDY2 = -0.1985;  // Fy load sensitivity
  double PDY3 = 4.0072;   // Fy camber sensitivity
  double PEY1 = 0.53284;  // Fy curvature
  double PEY2 = -0.33312;
  double PEY3 = 0.27748;
  double PEY4 = -0.016493;
  double PEY5 = 0;
  double PKY1 = -38.9068;  // Fy stiffness
  double PKY2 = 1.6336;
  double PKY3 = 0.62971;
  double PKY4 = 2;
  double PKY5 = 0;
  double PKY6 = -1;
  double PKY7 = 0;
  double PHY1 = 0;  // Fy horizontal shift
  double PHY2 = 0;
  double PVY1 = 0;  // Fy vertical shift
  double PVY2 = 0;
  double PVY3 = -3.2711;
  double PVY4 = -1.1442;
  double PPY1 = 0;  // Fy pressure effect
  double PPY2 = 0;
  double PPY3 = 0;
  double PPY4 = 0;
  double PPY5 = 0;

  // ---------------- rolling / aligning
  double QSY1 = -0.0309;  // Rolling resistance
  double QSY2 = -0.0921;
  double QSY7 = 0.85;  // Speed effect
  double QSY8 = -0.4;

  double QBZ1 = 10;     // Aligning stiffness
  double QCZ1 = 1.1;    // Aligning shape
  double QDZ1 = 0.12;   // Aligning peak
  double QDZ8 = -0.05;  // Aligning camber effect

  // ---------------- turnslip
  double PECP1 = 1.0;  // Camber reduction factors
  double PECP2 = 0;
  double PDXP1 = 0;  // Base reduction of force due to slip
  double PDXP2 = 0;
  double PDXP3 = 0;
  double PDXP4 = 0;
  double PDYP1 = 0;
  double PDYP2 = 0;
  double PDYP3 = 0;
  double PDYP4 = 0;
  double PKYP1 = 35;
  double PHYP1 = 0.2;  // base lateral shift
  double PHYP2 = 0.1;  // load-dependent shift
  double PHYP3 = 0.3;  // camber sensitivity
  double PHYP4 = 0.0;  // small smoothing offset
};

struct VehicleModelParams {
  TireParameters tire_parameters;
  float camber_scaling_factor;
  float effective_tire_r = 1.0;
  float distance_to_CG =
      1.0;  // MUST BE POSITIVE IF AHEAD OF CG AND NEGATIVE IF BEHIND OTHERWISE PACEJKA WILL BREAK
};

struct VehicleModelState {
  Velocity velocity;
  Force force;
  Wheels wheels_speed;
  double angular_speed;
  double steering_angle = 0.0;
  double yaw_rate;
};

/*
    Struct containing the tire radius, camber...
*/
