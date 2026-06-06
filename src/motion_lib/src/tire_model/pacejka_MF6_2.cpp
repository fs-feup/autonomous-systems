#include "motion_lib/tire_model/pacejka_MF6_2.hpp"

#include <algorithm>

// (4.E17)
double PacejkaMF6_2::calculate_SHx() const {
  return (car_parameters_->tire_parameters->PHX1 +
          car_parameters_->tire_parameters->PHX2 * internal_vals.dfz) *
         car_parameters_->tire_parameters->LHX;
}

// (4.E28)
double PacejkaMF6_2::calculate_SHy() const {
  double zeta4 = 1 + internal_vals.SHyp - internal_vals.SVyg / internal_vals.Kya_prime;

  return (car_parameters_->tire_parameters->PHY1 +
          car_parameters_->tire_parameters->PHY2 * internal_vals.dfz) *
             car_parameters_->tire_parameters->LHY +
         ((internal_vals.Kyg0 * internal_vals.gamma_star - internal_vals.SVyg) /
          (internal_vals.Kya + internal_vals.epsilon * std::copysign(1.0, internal_vals.Kya))) +
         zeta4 - 1;
}

Eigen::Vector4d PacejkaMF6_2::tire_forces(const TireInput& tire_input) {
  double Fx = 0;
  double Fy = 0;
  double MZ = 0;
  double MY = 0;

  // Calculate all the internal values needed for the calculations of forces and moments
  calculate_tire_state(tire_input.slip_angle, tire_input.slip_ratio, tire_input.vertical_load,
                       tire_input.vcx, tire_input.vcy, tire_input.yaw_rate,
                       tire_input.wheel_angular_speed, tire_input.camber_angle);

  // Low speed fade for slip shifts
  double speed = std::hypot(tire_input.vcx, tire_input.vcy);
  double shift_fade = std::clamp(speed / 0.1, 0.0, 1.0);

  // Shifts for longitudinal and lateral slip
  double SHx = calculate_SHx() * shift_fade;
  double SHy = calculate_SHy() * shift_fade;

  // (4.E20)
  double shifted_slip_a = internal_vals.alpha_star + SHy;
  // (4.E10)
  double force_slip_ratio =
      std::clamp(tire_input.slip_ratio, car_parameters_->tire_parameters->KPUMIN,
                 car_parameters_->tire_parameters->KPUMAX);
  double shifted_slip_r = force_slip_ratio + SHx;

  // Y parameter calculation
  double Dy = calculate_Dy(tire_input.vertical_load);
  double Cy = calculate_Cy(tire_input.vertical_load);
  double By = calculate_By(Dy, Cy);
  double Ey = calculate_Ey(shifted_slip_a);

  // Vertical shift for lateral force
  double SVy = calculate_SVy(tire_input.vertical_load) * shift_fade;

  // X parameter calculation
  double Dx = calculate_Dx(tire_input.vertical_load);
  double Cx = calculate_Cx(tire_input.vertical_load);
  double Bx = calculate_Bx(Dx, Cx);
  double Ex = calculate_Ex(shifted_slip_r);

  // Vertical shift for longitudinal force
  double SVx = calculate_SVx(tire_input.vertical_load) * shift_fade;

  // Pure slip calculations
  double Fx0 = calculate_pure_slip(Bx, Cx, Dx, Ex, shifted_slip_r, SVx);
  double Fy0 = calculate_pure_slip(By, Cy, Dy, Ey, shifted_slip_a, SVy);

  // Combined slip calculations
  Fx = calculate_combined_longitudinal(Fx0, force_slip_ratio);
  Fy = calculate_combined_lateral(Fy0, shifted_slip_r, force_slip_ratio, tire_input.vertical_load);

  // Aligining moment calculation
  MZ = calculate_combined_moment(Fx, Fy, force_slip_ratio, SHy, SVy, By, Cy, internal_vals.Vcx,
                                 tire_input.vertical_load);

  // Rolling resistance moment calculation
  MY = calculate_rolling_resistance_moment(tire_input.vertical_load, Fx, internal_vals.Vcx);

  return Eigen::Vector4d(Fx, Fy, MY, MZ);
}

// (4.E9 / 4.E19)
double PacejkaMF6_2::calculate_pure_slip(double B, double C, double D, double E,
                                         double shifted_slip, double SV) const {
  return D * sin(C * atan(B * shifted_slip - E * (B * shifted_slip - atan(B * shifted_slip)))) + SV;
}

/*
---------------------------
  D calculation:
    muy = (PDY1 + PDY2 .* dfz).*(1 + PPY3.*dpi + PPY4 .*dpi.^2).*(1
-PDY3.*gamma_star.^2).*LMUY_star; % (4.E23)
    Dy = muy.*Fz.*zeta2; % (4.E22)
------------------------------
*/

void PacejkaMF6_2::calculate_tire_state(double slip_angle, double slip_ratio, double vertical_load,
                                        double vcx, double vcy, double yaw_rate,
                                        double wheel_angular_speed, double camber_angle) {
  // Load related calculations
  // (4.E1) -> Assuming we have a tire with a different nominal load we approxiamte using scaling
  // factor LFZO The result is the adpated nominal load
  internal_vals.Fz0_prime =
      car_parameters_->tire_parameters->LFZO * car_parameters_->tire_parameters->FNOMIN;
  // (4.E2) -> Since we normalized the nominal load we need to normalize the change in vertical load
  internal_vals.dfz = (vertical_load - internal_vals.Fz0_prime) / internal_vals.Fz0_prime;
  // Camber reduction factor
  internal_vals.epsilong = car_parameters_->tire_parameters->PECP1 *
                           (1 + car_parameters_->tire_parameters->PECP2 * internal_vals.dfz);

  // Velocity related calculations using the same contact-patch velocity as slip calculations.
  internal_vals.Vc = sqrt(vcx * vcx + vcy * vcy);

  // Velocity of wheel contact center with safety factor for zero speed calculations
  internal_vals.Vc_prime = internal_vals.Vc + internal_vals.epsilon;

  internal_vals.Vcx = vcx;
  internal_vals.longitudinal_direction = std::copysign(1.0, internal_vals.Vcx);

  double alpha_arg = std::clamp(std::abs(internal_vals.Vcx) / internal_vals.Vc_prime, 0.0, 1.0);
  internal_vals.alpha_prime = acos(alpha_arg);

  // Camber related calculations
  // Total spin slip
  internal_vals.phi =
      (1 / internal_vals.Vc_prime) *
      (yaw_rate - (1 - internal_vals.epsilong) * wheel_angular_speed * sin(camber_angle));

  // Effective camber angle
  internal_vals.gamma_star = camber_angle * car_parameters_->tire_parameters->camber_scaling_factor;

  // Zetas
  internal_vals.zeta1 = calculate_Zeta1(slip_ratio);
  internal_vals.zeta2 = calculate_Zeta2(slip_angle);
  internal_vals.zeta3 = calculate_Zeta3();
  internal_vals.zeta7 = 1.0;  // Ignore turnslip effects on self-aligning moment
  internal_vals.zeta8 = 1.0;  // same as zeta7

  // Vertical load + Zetas
  //  Cornering stiffness at longitudinal slip zero
  internal_vals.Kya =
      car_parameters_->tire_parameters->PKY1 * internal_vals.Fz0_prime *
      (1 + car_parameters_->tire_parameters->PPY1 * internal_vals.dpi) *
      (1 - car_parameters_->tire_parameters->PKY3 * abs(internal_vals.gamma_star)) *
      sin(car_parameters_->tire_parameters->PKY4 *
          atan((vertical_load / internal_vals.Fz0_prime) /
               ((car_parameters_->tire_parameters->PKY2 + car_parameters_->tire_parameters->PKY5 *
                                                              internal_vals.gamma_star *
                                                              internal_vals.gamma_star) *
                (1 + car_parameters_->tire_parameters->PPY2 * internal_vals.dpi)))) *
      internal_vals.zeta3 * car_parameters_->tire_parameters->LKY;  // Ensure not dividing by zero

  internal_vals.Kya_prime =
      internal_vals.Kya + internal_vals.epsilon * std::copysign(1.0, internal_vals.Kya);

  // (4.E8) Special degressive friction factor
  internal_vals.LMUY_prime =
      car_parameters_->tire_parameters->Amu * car_parameters_->tire_parameters->LMUY /
      (1 + (car_parameters_->tire_parameters->Amu - 1) * car_parameters_->tire_parameters->LMUY);

  internal_vals.SVyg = vertical_load *
                       (car_parameters_->tire_parameters->PVY3 +
                        car_parameters_->tire_parameters->PVY4 * internal_vals.dfz) *
                       internal_vals.gamma_star * car_parameters_->tire_parameters->LKYC *
                       internal_vals.LMUY_prime * internal_vals.zeta2;
  // (4.E3)
  internal_vals.alpha_star = tan(slip_angle) * internal_vals.longitudinal_direction;
  internal_vals.muy =
      (car_parameters_->tire_parameters->PDY1 +
       car_parameters_->tire_parameters->PDY2 * internal_vals.dfz) *
      (1 + car_parameters_->tire_parameters->PPY3 * internal_vals.dpi +
       car_parameters_->tire_parameters->PPY4 * (internal_vals.dpi * internal_vals.dpi)) *
      (1 - car_parameters_->tire_parameters->PDY3 *
               (internal_vals.gamma_star * internal_vals.gamma_star)) *
      car_parameters_->tire_parameters
          ->LMUY;  // we can assume dpi is zero for now and ignore pressure variations

  internal_vals.Kyg0 = vertical_load *
                       (car_parameters_->tire_parameters->PKY6 +
                        car_parameters_->tire_parameters->PKY7 * internal_vals.dfz) *
                       (1 + car_parameters_->tire_parameters->PPY5 * internal_vals.dpi) *
                       car_parameters_->tire_parameters->LKYC;

  internal_vals.SHyp = calculate_SHyp(camber_angle);

  internal_vals.SVyk = calculate_SVyk(vertical_load, slip_ratio);
  internal_vals.kxk =
      vertical_load *
      (car_parameters_->tire_parameters->PKX1 +
       car_parameters_->tire_parameters->PKX2 * internal_vals.dfz) *
      exp(car_parameters_->tire_parameters->PKX3 * internal_vals.dfz) *
      (1 + car_parameters_->tire_parameters->PPX1 * internal_vals.dpi +
       car_parameters_->tire_parameters->PPX2 * (internal_vals.dpi * internal_vals.dpi)) *
      car_parameters_->tire_parameters->LKX;
  return;
}

double PacejkaMF6_2::calculate_Dy(double vertical_load) const {
  return internal_vals.muy * vertical_load * internal_vals.zeta2;
}

// (4.E12)
double PacejkaMF6_2::calculate_Dx(double vertical_load) const {
  if (vertical_load > 0) {
    // DANGEROUS: WE ARE NOT APPLYING A PENALTY FOR SLIP (SEE EQ 4.E13)
    double mux = ((car_parameters_->tire_parameters->PDX1 +
                   car_parameters_->tire_parameters->PDX2 * internal_vals.dfz) *
                  car_parameters_->tire_parameters->LMUX);
    return mux * vertical_load * internal_vals.zeta1;
  }
  return 0;
}

double PacejkaMF6_2::calculate_Cy(double vertical_load) const {
  if (vertical_load > 0) {
    return car_parameters_->tire_parameters->PCY1 * car_parameters_->tire_parameters->LCY;
  }
  return 0;
}

// (4.E11)
double PacejkaMF6_2::calculate_Cx(double vertical_load) const {
  if (vertical_load > 0) {
    return car_parameters_->tire_parameters->PCX1 * car_parameters_->tire_parameters->LCX;
  }
  return 0;
}

double PacejkaMF6_2::calculate_By(double Dy, double Cy) const {
  return internal_vals.Kya / (Cy * Dy + internal_vals.epsilon * std::copysign(1.0, Dy));
}

// (4.E16)
double PacejkaMF6_2::calculate_Bx(double Dx, double Cx) const {
  // Expanded (4.E15) that accounts for changes in pressure
  return internal_vals.kxk / (Cx * Dx + internal_vals.epsilon * std::copysign(1.0, Dx));
}

double PacejkaMF6_2::calculate_Ey(double SHy) const {
  double alpha_y = internal_vals.alpha_star + SHy;
  double Ey = (car_parameters_->tire_parameters->PEY1 +
               car_parameters_->tire_parameters->PEY2 * internal_vals.dfz) *
              (1 +
               car_parameters_->tire_parameters->PEY5 *
                   (internal_vals.gamma_star * internal_vals.gamma_star) -
               (car_parameters_->tire_parameters->PEY3 +
                car_parameters_->tire_parameters->PEY4 * internal_vals.gamma_star) *
                   std::copysign(1.0, alpha_y)) *
              car_parameters_->tire_parameters->LEY;
  return (Ey < 1) ? Ey : 1.0;  // E cannot be over 1
}

// (4.E14)
double PacejkaMF6_2::calculate_Ex(double shifted_slip_ratio) const {
  double Ex =
      (car_parameters_->tire_parameters->PEX1 +
       car_parameters_->tire_parameters->PEX2 * internal_vals.dfz +
       car_parameters_->tire_parameters->PEX3 * (internal_vals.dfz * internal_vals.dfz)) *
      (1 - car_parameters_->tire_parameters->PEX4 * std::copysign(1.0, shifted_slip_ratio)) *
      car_parameters_->tire_parameters->LEX;
  return (Ex < 1) ? Ex : 1.0;  // E cannot be over 1
}

double PacejkaMF6_2::calculate_SVy(double vertical_load) const {
  return vertical_load *
             (car_parameters_->tire_parameters->PVY1 +
              car_parameters_->tire_parameters->PVY2 * internal_vals.dfz) *
             car_parameters_->tire_parameters->LVY * internal_vals.LMUY_prime *
             internal_vals.zeta2 +
         internal_vals.SVyg;
}

// (4.E18)
double PacejkaMF6_2::calculate_SVx(double vertical_load) const {
  double LMUX_prime =
      car_parameters_->tire_parameters->Amu * car_parameters_->tire_parameters->LMUX /
      (1 + (car_parameters_->tire_parameters->Amu - 1) * car_parameters_->tire_parameters->LMUX);
  return vertical_load *
         (car_parameters_->tire_parameters->PVX1 +
          car_parameters_->tire_parameters->PVX2 * internal_vals.dfz) *
         car_parameters_->tire_parameters->LVX * LMUX_prime * internal_vals.zeta1;
}

double PacejkaMF6_2::calculate_combined_longitudinal(double Fx0, double slip_ratio) const {
  // (4.E53)
  double as = internal_vals.alpha_star + car_parameters_->tire_parameters->RHX1;
  // (4.E54)
  double Bxa = car_parameters_->tire_parameters->RBX1 *
               cos(atan(car_parameters_->tire_parameters->RBX2 * slip_ratio)) *
               car_parameters_->tire_parameters->LXAL;
  // (4.E56)
  double Exa = (car_parameters_->tire_parameters->REX1 +
                car_parameters_->tire_parameters->REX2 * internal_vals.dfz);
  // (4.E52)
  double Gxa0 = cos(car_parameters_->tire_parameters->RCX1 *
                    atan(Bxa * car_parameters_->tire_parameters->RHX1 -
                         Exa * (Bxa * car_parameters_->tire_parameters->RHX1 -
                                atan(Bxa * car_parameters_->tire_parameters->RHX1))));
  // (4.E51)
  double Gxa = cos(car_parameters_->tire_parameters->RCX1 *
                   atan(Bxa * as - Exa * (Bxa * as - atan(Bxa * as)))) /
               Gxa0;
  // (4.E50)
  return Fx0 * Gxa;
}

double PacejkaMF6_2::calculate_combined_lateral(double Fy0, double shifted_slip_ratio,
                                                double slip_ratio, double vertical_load) const {
  // Calculation of SVyk
  // (4.E67)
  double DVyk = internal_vals.muy * vertical_load *
                (car_parameters_->tire_parameters->RVY1 +
                 car_parameters_->tire_parameters->RVY2 * internal_vals.dfz +
                 car_parameters_->tire_parameters->RVY3 * internal_vals.gamma_star) *
                cos(atan(car_parameters_->tire_parameters->RVY4 * internal_vals.alpha_star)) *
                internal_vals.zeta2;
  // (4.E66)
  double SVyk = DVyk * sin(car_parameters_->tire_parameters->RVY5 *
                           atan(car_parameters_->tire_parameters->RVY6 * slip_ratio));
  // Calculation of Gyk
  // (4.E64)
  double Eyk = car_parameters_->tire_parameters->REY1 +
               car_parameters_->tire_parameters->REY2 * internal_vals.dfz;
  Eyk = (Eyk < 1) ? Eyk : 1.0;  // E cannot be over 1
  // (4.E65)
  double SHyk = car_parameters_->tire_parameters->RHY1 +
                car_parameters_->tire_parameters->RHY2 * internal_vals.dfz;
  // (4.E62)
  double Byk = car_parameters_->tire_parameters->RBY1 *
               cos(atan(car_parameters_->tire_parameters->RBY2 *
                        (internal_vals.alpha_star - car_parameters_->tire_parameters->RBY3))) *
               car_parameters_->tire_parameters->LYKA;
  // (4.E60)
  double Gyk0 = cos(car_parameters_->tire_parameters->RCY1 *
                    atan(Byk * SHyk - Eyk * (Byk * SHyk - atan(Byk * SHyk))));
  // (4.E59)
  double Gyk = cos(car_parameters_->tire_parameters->RCY1 *
                   atan(Byk * shifted_slip_ratio -
                        Eyk * (Byk * shifted_slip_ratio - atan(Byk * shifted_slip_ratio)))) /
               Gyk0;
  // (4.E58)
  return Fy0 * Gyk + SVyk;
}

// This function is never used (Im stupid and thought that the pure slip aligning moment was needed
// for the combined slip calculation) either way it is here if we need it later for some reason.
double PacejkaMF6_2::calculate_pure_moment(double Fy0, double normal_load, double SHy, double SVy,
                                           double By, double Cy,
                                           double longitudinal_velocity) const {
  // MZ0_prime calculation
  // ------------------ at calculation
  // (4.E35)
  double SHt = car_parameters_->tire_parameters->QHZ1 +
               car_parameters_->tire_parameters->QHZ2 * internal_vals.dfz +
               (car_parameters_->tire_parameters->QHZ3 +
                car_parameters_->tire_parameters->QHZ4 * internal_vals.dfz) *
                   internal_vals.gamma_star;
  // (4.E34)
  double at = internal_vals.alpha_star + SHt;
  // ------------------ Bt calculation
  // (4.E40)
  double Bt = (car_parameters_->tire_parameters->QBZ1 +
               car_parameters_->tire_parameters->QBZ2 * internal_vals.dfz +
               car_parameters_->tire_parameters->QBZ3 * internal_vals.dfz * internal_vals.dfz) *
              (1 + car_parameters_->tire_parameters->QBZ4 * internal_vals.gamma_star +
               car_parameters_->tire_parameters->QBZ5 * abs(internal_vals.gamma_star)) *
              internal_vals.Kya / car_parameters_->tire_parameters->LMUY;
  // ------------------ Dt calculation
  // (4.91)
  double zeta5 = cos(atan(car_parameters_->tire_parameters->QDTP1 *
                          car_parameters_->tire_parameters->effective_tire_r * internal_vals.phi));
  // (4.E42) -> expanded to account for pressure changes (dpi = 0 to ignore)
  double Dt0 = normal_load *
               (car_parameters_->tire_parameters->effective_tire_r / internal_vals.Fz0_prime) *
               (car_parameters_->tire_parameters->QDZ1 +
                car_parameters_->tire_parameters->QDZ2 * internal_vals.dfz) *
               (1 - car_parameters_->tire_parameters->PPZ1) *
               car_parameters_->tire_parameters->LTR * std::copysign(1.0, longitudinal_velocity);
  // (4.E43)
  double Dt = Dt0 *
              (1 + car_parameters_->tire_parameters->QDZ3 * internal_vals.gamma_star +
               car_parameters_->tire_parameters->QDZ4 *
                   (internal_vals.gamma_star * internal_vals.gamma_star)) *
              zeta5;
  // ------------------ Et calculation
  // (4.E44)
  double Et = (car_parameters_->tire_parameters->QEZ1 +
               car_parameters_->tire_parameters->QEZ2 * internal_vals.dfz +
               car_parameters_->tire_parameters->QEZ3 * internal_vals.dfz * internal_vals.dfz) *
              (1 + (car_parameters_->tire_parameters->QEZ4 * internal_vals.gamma_star +
                    car_parameters_->tire_parameters->QEZ5 * internal_vals.gamma_star *
                        internal_vals.gamma_star) *
                       (2 / M_PI) * atan(Bt * car_parameters_->tire_parameters->QCZ1 * at));
  Et = (Et < 1) ? Et : 1.0;  // Et cannot be over 1
  // ------------------
  // (4.E6)

  // (4.E33)
  double t0 =
      Dt *
      cos(car_parameters_->tire_parameters->QCZ1 * atan(Bt * at - Et * (Bt * at - atan(Bt * at)))) *
      cos(internal_vals.alpha_prime);
  // (4.E32)
  double MZ0_prime = -t0 * Fy0;
  // MZr0 calculation
  // (4.E47)
  double Dr = normal_load * car_parameters_->tire_parameters->effective_tire_r *
                  ((car_parameters_->tire_parameters->QDZ6 +
                    car_parameters_->tire_parameters->QDZ7 * internal_vals.dfz) *
                       car_parameters_->tire_parameters->LRES * internal_vals.zeta2 +
                   (car_parameters_->tire_parameters->QDZ8 +
                    car_parameters_->tire_parameters->QDZ9 * internal_vals.dfz) *
                       internal_vals.gamma_star * car_parameters_->tire_parameters->LKZC *
                       internal_vals.zeta0) *
                  cos(internal_vals.alpha_prime) * car_parameters_->tire_parameters->LMUY *
                  std::copysign(1.0, longitudinal_velocity) +
              internal_vals.zeta8 - 1;
  // (4.E45)
  double Br = car_parameters_->tire_parameters->QBZ10 * By * Cy;
  // (4.E38)
  double SHf = SHy + SVy / (internal_vals.Kya +
                            internal_vals.epsilon * std::copysign(1.0, internal_vals.Kya));
  // (4.E37)
  double ar = internal_vals.alpha_star + SHf;
  // (4.E36) -> currently we set zeta7 to 1 to ignore turnslip effects, but if we want to consider
  // them we need to calculate zeta7
  double MZr0 = Dr * cos(internal_vals.zeta7 * atan(Br * ar));
  // (4.E31)
  return MZ0_prime + MZr0;
}

double PacejkaMF6_2::calculate_combined_moment(double Fx, double Fy, double slip_ratio, double SHy,
                                               double SVy, double By, double Cy,
                                               double longitudinal_velocity,
                                               double normal_load) const {
  // MZ_prime calculation
  // ------------------ at_eq calculation
  // (4.E35)
  double SHt = car_parameters_->tire_parameters->QHZ1 +
               car_parameters_->tire_parameters->QHZ2 * internal_vals.dfz +
               (car_parameters_->tire_parameters->QHZ3 +
                car_parameters_->tire_parameters->QHZ4 * internal_vals.dfz) *
                   internal_vals.gamma_star;
  // (4.E34)
  double at = internal_vals.alpha_star + SHt;
  // (4.E77)
  double at_eq =
      sqrt(at * at + (internal_vals.kxk / internal_vals.Kya_prime) *
                         (internal_vals.kxk / internal_vals.Kya_prime) * slip_ratio * slip_ratio) *
      std::copysign(1.0, at);
  // ------------------ Bt calculation
  // (4.E40)
  double Bt = (car_parameters_->tire_parameters->QBZ1 +
               car_parameters_->tire_parameters->QBZ2 * internal_vals.dfz +
               car_parameters_->tire_parameters->QBZ3 * internal_vals.dfz * internal_vals.dfz) *
              (1 + car_parameters_->tire_parameters->QBZ4 * internal_vals.gamma_star +
               car_parameters_->tire_parameters->QBZ5 * abs(internal_vals.gamma_star)) *
              internal_vals.Kya / car_parameters_->tire_parameters->LMUY;
  // ------------------ Dt calculation
  // (4.91)
  double zeta5 = cos(atan(car_parameters_->tire_parameters->QDTP1 *
                          car_parameters_->tire_parameters->effective_tire_r * internal_vals.phi));
  // (4.E42) -> expanded to account for pressure changes (dpi = 0 to ignore)
  double Dt0 = normal_load *
               (car_parameters_->tire_parameters->effective_tire_r / internal_vals.Fz0_prime) *
               (car_parameters_->tire_parameters->QDZ1 +
                car_parameters_->tire_parameters->QDZ2 * internal_vals.dfz) *
               (1 - car_parameters_->tire_parameters->PPZ1) *
               car_parameters_->tire_parameters->LTR * internal_vals.longitudinal_direction;
  // (4.E43)
  double Dt = Dt0 *
              (1 + car_parameters_->tire_parameters->QDZ3 * internal_vals.gamma_star +
               car_parameters_->tire_parameters->QDZ4 *
                   (internal_vals.gamma_star * internal_vals.gamma_star)) *
              zeta5;
  // ------------------ Et calculation
  // (4.E44)
  double Et = (car_parameters_->tire_parameters->QEZ1 +
               car_parameters_->tire_parameters->QEZ2 * internal_vals.dfz +
               car_parameters_->tire_parameters->QEZ3 * internal_vals.dfz * internal_vals.dfz) *
              (1 + (car_parameters_->tire_parameters->QEZ4 * internal_vals.gamma_star +
                    car_parameters_->tire_parameters->QEZ5 * internal_vals.gamma_star *
                        internal_vals.gamma_star) *
                       (2 / M_PI) * atan(Bt * car_parameters_->tire_parameters->QCZ1 * at));
  Et = (Et < 1) ? Et : 1.0;  // Et cannot be over 1
  // ------------------ t calculation
  // (4.E73)
  double t = Dt *
             cos(car_parameters_->tire_parameters->QCZ1 *
                 atan(Bt * at_eq - Et * (Bt * at_eq - atan(Bt * at_eq)))) *
             cos(internal_vals.alpha_prime);
  // (4.E74)
  double Fy_prime = Fy - internal_vals.SVyk;
  // (4.E72)
  double MZ_prime = -t * Fy_prime;
  // MZr calculation
  // (4.E47)
  double Dr = normal_load * car_parameters_->tire_parameters->effective_tire_r *
                  ((car_parameters_->tire_parameters->QDZ6 +
                    car_parameters_->tire_parameters->QDZ7 * internal_vals.dfz) *
                       car_parameters_->tire_parameters->LRES * internal_vals.zeta2 +
                   (car_parameters_->tire_parameters->QDZ8 +
                    car_parameters_->tire_parameters->QDZ9 * internal_vals.dfz) *
                       internal_vals.gamma_star * car_parameters_->tire_parameters->LKZC *
                       internal_vals.zeta0) *
                  cos(internal_vals.alpha_prime) * car_parameters_->tire_parameters->LMUY *
                  internal_vals.longitudinal_direction +
              internal_vals.zeta8 - 1;
  // (4.E45)
  double Br = car_parameters_->tire_parameters->QBZ10 * By * Cy;
  // (4.E38)
  double SHf = SHy + SVy / (internal_vals.Kya +
                            internal_vals.epsilon * std::copysign(1.0, internal_vals.Kya));
  // (4.E37)
  double ar = internal_vals.alpha_star + SHf;
  // (4.E78)
  double ar_eq =
      sqrt(ar * ar + (internal_vals.kxk / internal_vals.Kya_prime) *
                         (internal_vals.kxk / internal_vals.Kya_prime) * slip_ratio * slip_ratio) *
      std::copysign(1.0, ar);
  // (4.E75)
  double MZr = Dr * cos(internal_vals.zeta7 * atan(Br * ar_eq));
  // (4.E71)
  // s calculation
  // (4.E76)
  double s = car_parameters_->tire_parameters->effective_tire_r *
             (car_parameters_->tire_parameters->SSZ1 +
              car_parameters_->tire_parameters->SSZ2 * (Fy / internal_vals.Fz0_prime) +
              (car_parameters_->tire_parameters->SSZ3 +
               car_parameters_->tire_parameters->SSZ4 * internal_vals.dfz) *
                  internal_vals.gamma_star) *
             car_parameters_->tire_parameters->LS;
  return MZ_prime + MZr + s * Fx;
}

double PacejkaMF6_2::calculate_rolling_resistance_moment(double vertical_load, double Fx,
                                                         double vx) const {
  double Fz0 = internal_vals.Fz0_prime;
  double unloaded_radius = car_parameters_->tire_parameters->UNLOADED_RADIUS;
  double QSY1 = car_parameters_->tire_parameters->QSY1;
  double QSY2 = car_parameters_->tire_parameters->QSY2;
  double QSY3 = car_parameters_->tire_parameters->QSY3;
  double QSY4 = car_parameters_->tire_parameters->QSY4;

  // Standard Pacejka reference velocity is 16.666 m/s (60 km/h)
  const double Vref = 16.666;

  // (4.E85) gives the coefficient magnitude. Sign it with Vcx so the moment opposes rolling
  // in both forward and reverse motion.
  double My_magnitude = std::abs(
      unloaded_radius * vertical_load *
      (QSY1 + QSY2 * (Fx / Fz0) + QSY3 * std::abs(vx / Vref) + QSY4 * std::pow(vx / Vref, 4)));
  double direction = 2.0 / M_PI * std::atan(10.0 * vx);

  return My_magnitude * direction;
}

double PacejkaMF6_2::calculate_Zeta1(double slip_ratio) const {
  // (4.106)
  double Bxp = car_parameters_->tire_parameters->PDXP1 *
               (1 + car_parameters_->tire_parameters->PDXP2 * internal_vals.dfz) *
               cos(atan(car_parameters_->tire_parameters->PDXP3 * slip_ratio));
  // (4.105)
  return cos(atan(Bxp * car_parameters_->tire_parameters->effective_tire_r * internal_vals.phi));
}

double PacejkaMF6_2::calculate_Zeta2(double slip_angle) const {
  double Byp = car_parameters_->tire_parameters->PDYP1 *
               (1 + car_parameters_->tire_parameters->PDYP2 * internal_vals.dfz) *
               cos(atan(car_parameters_->tire_parameters->PDYP3 * tan(slip_angle)));
  return cos(
      atan(Byp *
           (car_parameters_->tire_parameters->UNLOADED_RADIUS * abs(internal_vals.phi) +
            car_parameters_->tire_parameters->PDYP4 *
                sqrt(car_parameters_->tire_parameters->UNLOADED_RADIUS * abs(internal_vals.phi)))));
}

double PacejkaMF6_2::calculate_Zeta3() const {
  // (4.79)
  return cos(atan(
      car_parameters_->tire_parameters->PKYP1 * car_parameters_->tire_parameters->effective_tire_r *
      car_parameters_->tire_parameters->effective_tire_r * internal_vals.phi * internal_vals.phi));
}

double PacejkaMF6_2::calculate_SHyp(double camber_angle) const {
  // MF 6.1 and 6.2
  double KyRp0 = internal_vals.Kyg0 / (1 - internal_vals.epsilong);

  double Kya0 =
      car_parameters_->tire_parameters->PKY1 * internal_vals.Fz0_prime *
      (1 + car_parameters_->tire_parameters->PPY1 * internal_vals.dpi) *
      sin(car_parameters_->tire_parameters->PKY4 *
          (1 + car_parameters_->tire_parameters->PPX3 * internal_vals.dpi +
           car_parameters_->tire_parameters->PPX4 * (internal_vals.dpi * internal_vals.dpi)) *
          (1 - car_parameters_->tire_parameters->PDX3 * (camber_angle * camber_angle))) *
      car_parameters_->tire_parameters->LMUX;
  double Kyao_prime = Kya0 + internal_vals.epsilon * std::copysign(1.0, Kya0);

  double DHyp = (car_parameters_->tire_parameters->PHYP2 +
                 car_parameters_->tire_parameters->PHYP3 * internal_vals.dfz) *
                internal_vals.longitudinal_direction;

  double BHyp = KyRp0 / (car_parameters_->tire_parameters->PHYP1 * DHyp * Kyao_prime);

  double EHyp =
      (car_parameters_->tire_parameters->PHYP4 > 1) ? 1 : car_parameters_->tire_parameters->PHYP4;

  return DHyp *
         sin(car_parameters_->tire_parameters->PHYP1 *
             atan(BHyp * car_parameters_->tire_parameters->effective_tire_r * internal_vals.phi -
                  EHyp * (BHyp * car_parameters_->tire_parameters->effective_tire_r *
                              internal_vals.phi -
                          atan(BHyp * car_parameters_->tire_parameters->effective_tire_r *
                               internal_vals.phi)))) *
         internal_vals.longitudinal_direction;
}
double PacejkaMF6_2::calculate_SVyk(double vertical_load, double slip_ratio) const {
  // (4.E67)
  double DVyk = internal_vals.muy * vertical_load *
                (car_parameters_->tire_parameters->RVY1 +
                 car_parameters_->tire_parameters->RVY2 * internal_vals.dfz +
                 car_parameters_->tire_parameters->RVY3 * internal_vals.gamma_star) *
                cos(atan(car_parameters_->tire_parameters->RVY4 * internal_vals.alpha_star)) *
                internal_vals.zeta2;
  // (4.E66)
  return DVyk * sin(car_parameters_->tire_parameters->RVY5 *
                    atan(car_parameters_->tire_parameters->RVY6 * slip_ratio));
}
