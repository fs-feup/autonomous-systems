#include "motion_lib/tire_model/pacejka_MF6_2.hpp"


//IMPORTANT: WE MIGHT NEED TO SUBTRACT THE STEERING ANGLE FROM THE FRONT WHEELS (THE WHEEL COULD BE MOVING 5 deg because we turned 5 deg and then the net slip should be)
// float MF6_2::calculateSlipAngle() const {
//   double v_x = vehicle_model_state.velocities.velocity_x;
//   if (v_x == 0.0) v_x = 0.1;
//   float lateral_velocity_wheel = vehicle_model_state.velocities.velocity_y +
//                     (internal_vals.distance_to_CG * vehicle_model_state.yaw_rate);
//   return atan2(
//       lateral_velocity_wheel,
//       v_x);  // atan2 permite cobrir todos os ranges de angulo e não apenas de -pi/2 até pi/2
// }

// float MF6_2::calculateSlipRatio() const {
//   double v_x = vehicle_model_state.velocities.velocity_x;
//   if (v_x == 0.0) v_x = 0.1;
//   float numerator =
//       (car_parameters_->tire_parameters->effective_tire_r * vehicle_model_state.angular_speed) -
//       v_x;
//   return numerator / v_x;
// }

// (4.E17)
float MF6_2::calculateSHx() const{
  return (car_parameters_->tire_parameters->PHX1 + car_parameters_->tire_parameters->PHX2 * internal_vals.dfz)*car_parameters_->tire_parameters->LHX;
} 

// (4.E28)
float MF6_2::calculateSHy() const{
  double Kya_prime = internal_vals.Kya + internal_vals.epsilon * sign(internal_vals.Kya);
  double zeta4 = 1 + SHyp - internal_vals.SVyg / Kya_prime;
  return (car_parameters_->tire_parameters->PHY1 +
                car_parameters_->tire_parameters->PHY2 * internal_vals.dfz) *
                   car_parameters_->tire_parameters->LHY +
               ((Kyg0 * internal_vals.gamma_star - internal_vals.SVyg) /
                (internal_vals.Kya + internal_vals.epsilon * sign(internal_vals.Kya))) +
               zeta4 - 1;
}


std::pair<double, double> MF6_2::tire_forces(double slip_angle, double slip_ratio,
                                                      double vertical_load) const {
  if (calculateTireState(shifted_slip_a, shifted_slip_r, vertical_load)){
    float SHx = calculateSHx();
    float SHy = calculateSHy();
    // (4.E20)
    float shifted_slip_a = internal_vals.alpha_star + SHy;
    // (4.E10) 
    float shifted_slip_r = slip_ratio + SHx;
    // Y parameter calculation
    float Dy = calculateDy(vertical_load);
    float Cy = calculateCy(vertical_load);
    float By = calculateBy(Dy, Cy);
    float Ey = calculateEy(shifted_slip_a, vertical_load);
    float SVy = calculateSVy(vertical_load);
    // X parameter calculation
    float Dx = calculateDx(vertical_load);
    float Cx = calculateCx(vertical_load);
    float Bx = calculateBx(Dx, Cx, vertical_load);
    float Ex = calculateEx(shifted_slip_r);
    float SVx = calculateSVx(vertical_load);
    // Aligining moment calculation
    double MZ0 = calculatePureMoment(SHy);

    // Pure slip calculations
    double Fx0 = calculatePureSlip(Bx, Cx, Dx, Ex, shifted_slip_r, SVx);
    double Fy0 = calculatePureSlip(By, Cy, Dy, Ey, shifted_slip_a, SVy);
    // Combined slip calculations
    double Fx = calculateCombinedLongitudinal(Fx0);
    double Fy = calculateCombinedLateral(Fy0, shifted_slip_r, slip_ratio, vertical_load);
  }
  else{
    double Fx = 0;
    double Fy = 0;
  }

  // After the definition of the input of tire models this needs to be deleted
  return {Fx, Fy};
}

// (4.E9 / 4.E19) 
float MF6_2::calculatePureSlip(float B, float C, float D, float E, float shifted_slip,
                                       float SV) const {
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

bool MF6_2::calculateTireState(float slip_angle, float slip_ratio, double vertical_load) {
  // (4.E1) -> Assuming we have a tire with a different nominal load we approxiamte using scaling factor LFZO
  // The result is the adpated nominal load
  internal_vals.Fz0_prime = car_parameters_->tire_parameters->LFZO * car_parameters_->tire_parameters->FNOMIN;
  // (4.E2) -> Since we normalized the nominal load we need to normalize the change in vertical load
  internal_vals.dfz =
      (vertical_load - internal_vals.Fz0_prime) / internal_vals.Fz0_prime;

  internal_vals.epsilong = car_parameters_->tire_parameters->PECP1 *
                           (1 + car_parameters_->tire_parameters->PECP2 * internal_vals.dfz);
  // Simple velocity of wheel contact center
  internal_vals.Vc =
      sqrt(vehicle_model_state.velocities.velocity_x * vehicle_model_state.velocities.velocity_x +
           vehicle_model_state.velocities.velocity_y * vehicle_model_state.velocities.velocity_y);
  // Velocity of wheel contact center with safety factor for zero speed calculations
  interval_vals.Vc_prime = internal_vals.Vc + internal_vals.epsilon * sign(internal_vals.Vc);
  internal_vals.phi = (1 / internal_vals.Vc_prime) *
                      (vehicle_model_state.yaw_rate - (1 - internal_vals.epsilong) *
                                                          vehicle_model_state.angular_speed *
                                                          sin(internal_vals.camber_angle));
  internal_vals.gamma_star =
      internal_vals.camber_angle * car_parameters_->tire_parameters->camber_scaling_factor;
  double Byp = car_parameters_->tire_parameters->PDYP1 *
               (1 + car_parameters_->tire_parameters->PDYP2 * internal_vals.dfz) *
               cos(atan(car_parameters_->tire_parameters->PDYP3 * tan(slip_angle)));
  internal_vals.zeta2 = cos(
      atan(Byp *
           (car_parameters_->tire_parameters->UNLOADED_RADIUS * abs(internal_vals.phi) +
            car_parameters_->tire_parameters->PDYP4 *
                sqrt(car_parameters_->tire_parameters->UNLOADED_RADIUS * abs(internal_vals.phi)))));
  internal_vals.zeta3 = cos(atan(car_parameters_->tire_parameters->PKYP1 *
                                 (car_parameters_->tire_parameters->effective_tire_r *
                                  car_parameters_->tire_parameters->effective_tire_r) *
                                 (internal_vals.phi * internal_vals.phi)));
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
      internal_vals.zeta3 * car_parameters_->tire_parameters->LKY;
  internal_vals.Vcx =
      (vehicle_model_state.velocities.velocity_x * cos(vehicle_model_state.steering_angle)) +
      (vehicle_model_state.velocities.velocity_y +
       vehicle_model_state.yaw_rate * internal_vals.distance_to_CG) *
          sin(vehicle_model_state.steering_angle);
  // (4.106)
  double Bxp = car_parameters_->tire_parameters->PDXP1 *
               (1 + car_parameters_->tire_parameters->PDXP2 * internal_vals.dfz) *
               cos(atan(car_parameters_->tire_parameters->PDXP3 * slip_ratio));
  // (4.105)
  internal_vals.zeta1 =
      cos(atan(Bxp * car_parameters_->tire_parameters->effective_tire_r * internal_vals.phi));
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
  internal_vals.alpha_star = tan(slip_angle) * sign(internal_vals.Vcx);
  internal_vals.muy = (car_parameters_->tire_parameters->PDY1 +
                car_parameters_->tire_parameters->PDY2 * internal_vals.dfz) *
               (1 + car_parameters_->tire_parameters->PPY3 * internal_vals.dpi +
                car_parameters_->tire_parameters->PPY4 * (internal_vals.dpi * internal_vals.dpi)) *
               (1 - car_parameters_->tire_parameters->PDY3 *
                        (internal_vals.gamma_star * internal_vals.gamma_star)) *
               car_parameters_->tire_parameters->LMUY;  // we can assume dpi is zero for now and ignore pressure variations
  return true;
}

float MF6_2::calculateDy(double vertical_load) const {
  return internal_vals.muy * vertical_load * internal_vals.zeta2;
}

// (4.E12)
float MF6_2::calculateDx(double vertical_load) const {
  if (vertical_load > 0) {
    // DANGEROUS: WE ARE NOT APPLYING A PENALTY FOR SLIP (SEE EQ 4.E13)
    double mux =
        ((car_parameters_->tire_parameters->PDX1 +
         car_parameters_->tire_parameters->PDX2 * internal_vals.dfz) *
         car_parameters_->tire_parameters->LMUX );
    return mux * vertical_load * internal_vals.zeta1;
  }
  return 0;
}

  // (4.E11)
float MF6_2::calculateCx(double vertical_load) const {
  if (vertical_load > 0) {
    return car_parameters_->tire_parameters->PCX1 * car_parameters_->tire_parameters->LCX;
  }
  return 0;
}

float MF6_2::calculateCy(double vertical_load) const {
  if (vertical_load > 0) {
    return car_parameters_->tire_parameters->PCY1 * car_parameters_->tire_parameters->LCY;
  }
  return 0;
}

// (4.E16)
float MF6_2::calculateBx(float Dx, float Cx, double vertical_load) const {
  // Expanded (4.E15) that accounts for changes in pressure
  double Kxk = vertical_load *
               (car_parameters_->tire_parameters->PKX1 +
                car_parameters_->tire_parameters->PKX2 * internal_vals.dfz) *
               exp(car_parameters_->tire_parameters->PKX3 * internal_vals.dfz) *
               (1 + car_parameters_->tire_parameters->PPX1 * internal_vals.dpi +
                car_parameters_->tire_parameters->PPX2 * (internal_vals.dpi * internal_vals.dpi)) *
               car_parameters_->tire_parameters->LKX;
  return Kxk / (Cx * Dx + internal_vals.epsilon * sign(Dx));
}

float MF6_2::calculateBy(float Dy, float Cy) const {  
  return internal_vals.Kya / (Cy * Dy + internal_vals.epsilon * sign(Dy));
}

// (4.E14)
float MF6_2::calculateEx(double shifted_slip_ratio) const {
  double Ex = (car_parameters_->tire_parameters->PEX1 +
               car_parameters_->tire_parameters->PEX2 * internal_vals.dfz +
               car_parameters_->tire_parameters->PEX3 * (internal_vals.dfz * internal_vals.dfz)) *
              (1 - car_parameters_->tire_parameters->PEX4 * sign(shifted_slip_ratio)) *
              car_parameters_->tire_parameters->LEX;
  return (Ex < 1) ? Ex : 1.0;  // E cannot be over 1
}

float MF6_2::calculateEy(double slip_angle, double vertical_load) const {
  // MF 6.1 and 6.2
  double Kyg0 = vertical_load *
                (car_parameters_->tire_parameters->PKY6 +
                 car_parameters_->tire_parameters->PKY7 * internal_vals.dfz) *
                (1 + car_parameters_->tire_parameters->PPY5 * internal_vals.dpi) *
                car_parameters_->tire_parameters->LKYC;
  double DHyp = (car_parameters_->tire_parameters->PHYP2 +
                 car_parameters_->tire_parameters->PHYP3 * internal_vals.dfz) *
                sign(internal_vals.Vcx);
  double KyRp0 = Kyg0 / (1 - internal_vals.epsilong);
  double Kya0 = car_parameters_->tire_parameters->PKY1 * internal_vals.Fz0_prime *
                (1 + car_parameters_->tire_parameters->PPY1 * internal_vals.dpi) *
                sin(car_parameters_->tire_parameters->PKY4 *        (1 + car_parameters_->tire_parameters->PPX3 * internal_vals.dpi +
         car_parameters_->tire_parameters->PPX4 * (internal_vals.dpi * internal_vals.dpi)) *
        (1 - car_parameters_->tire_parameters->PDX3 *
                 (internal_vals.camber_angle * internal_vals.camber_angle)) *
        car_parameters_->tire_parameters->LMUX;
                    atan((vertical_load / internal_vals.Fz0_prime) /
                         (car_parameters_->tire_parameters->PKY2 *
                          (1 + car_parameters_->tire_parameters->PPY2 * internal_vals.dpi)))) *
                internal_vals.zeta3 * car_parameters_->tire_parameters->LKY;
  double Kyao_prime = Kya0 + internal_vals.epsilon * sign(Kya0);
  double BHyp = KyRp0 / (car_parameters_->tire_parameters->PHYP1 * DHyp * Kyao_prime);
  double EHyp =
      (car_parameters_->tire_parameters->PHYP4 > 1) ? 1 : car_parameters_->tire_parameters->PHYP4;
  double SHyp =
      DHyp *
      sin(car_parameters_->tire_parameters->PHYP1 *
          atan(BHyp * car_parameters_->tire_parameters->effective_tire_r * internal_vals.phi -
               EHyp *
                   (BHyp * car_parameters_->tire_parameters->effective_tire_r * internal_vals.phi -
                    atan(BHyp * car_parameters_->tire_parameters->effective_tire_r *
                         internal_vals.phi)))) *
      sign(internal_vals.Vcx);

  double alpha_y = internal_vals.alpha_star + SHy;
  double Ey = (car_parameters_->tire_parameters->PEY1 +
               car_parameters_->tire_parameters->PEY2 * internal_vals.dfz) *
              (1 +
               car_parameters_->tire_parameters->PEY5 *
                   (internal_vals.gamma_star * internal_vals.gamma_star) -
               (car_parameters_->tire_parameters->PEY3 +
                car_parameters_->tire_parameters->PEY4 * internal_vals.gamma_star) *
                   sign(alpha_y)) *
              car_parameters_->tire_parameters->LEY;
  return (Ey < 1) ? Ey : 1.0;  // E cannot be over 1
}

// (4.E18)
float MF6_2::calculateSVx(double vertical_load) const {
  double LMUX_prime =
      car_parameters_->tire_parameters->Amu * car_parameters_->tire_parameters->LMUX /
      (1 + (car_parameters_->tire_parameters->Amu - 1) * car_parameters_->tire_parameters->LMUX);
  return vertical_load *
         (car_parameters_->tire_parameters->PVX1 +
          car_parameters_->tire_parameters->PVX2 * internal_vals.dfz) *
         car_parameters_->tire_parameters->LVX * LMUX_prime * internal_vals.zeta1;
}

float MF6_2::calculateSVy(double vertical_load) const {
  return vertical_load *
             (car_parameters_->tire_parameters->PVY1 +
              car_parameters_->tire_parameters->PVY2 * internal_vals.dfz) *
             car_parameters_->tire_parameters->LVY * internal_vals.LMUY_prime *
             internal_vals.zeta2 +
         internal_vals.SVyg;
}


float MF6_2::calculateCombinedLongitudinal(double Fx0) const {
  // (4.E53)
  float as = internal_vals.alpha_star + car_parameters_->tire_parameters->RHX1;
  // (4.E54)
  float Bxa = car_parameters_->tire_parameters->RBX1 * cos(atan(car_parameters_->tire_parameters->RBX2 * slip_ratio)) * car_parameters_->tire_parameters->LXAL;
  // (4.E56)
  float Exa = (car_parameters_->tire_parameters->REX1 + car_parameters_->tire_parameters->REX2 * internal_vals.dfz);
  // (4.E52)
  float Gxa0 = cos(car_parameters_->tire_parameters->RCX1 * atan(Bxa * car_parameters_->tire_parameters->RHX1 - Exa * (Bxa * car_parameters_->tire_parameters->RHX1 - atan(Bxa * car_parameters_->tire_parameters->RHX1))));
  // (4.E51)
  float Gxa = cos(Cxa * atan(Bxa * as - Exa * (Bxa * as - atan(Bxa * as)))) / Gxa0;
  // (4.E50)
  return Fx0 * Gxa;
}

float MF6_2::calculateCombinedLateral(double Fy0 , double shifted_slip_ratio, double slip_ratio , double vertical_load) const {
// Calculation of SVyk
  // (4.E67)
  double DVyk = internal_vals.muy * vertical_load *(car_parameters_->tire_parameters->RVY1 + car_parameters_->tire_parameters->RVY2 * internal_vals.dfz + car_parameters_->tire_parameters->RVY3 * internal_vals.gamma_star) * cos(atan(car_parameters_->tire_parameters->RVY4 * internal_vals.alpha_star)) * internal_vals.zeta2;
  // (4.E66)
  double SVyk = DVyk * sin(car_parameters_->tire_parameters->RVY5 * atan( car_parameters_->tire_parameters->RVY6 * slip_ratio));
// Calculation of Gyk
  // (4.E64)
  double Eyk = car_parameters_->tire_parameters->REY1 + car_parameters_->tire_parameters->REY2 * internal_vals.dfz;
  Eyk = (Eyk < 1) ? Eyk : 1.0;  // E cannot be over 1
  // (4.E65)
  double SHyk = car_parameters_->tire_parameters->RHY1 + car_parameters_->tire_parameters->RHY2 * internal_vals.dfz;
  // (4.E62)
  double Byk = car_parameters_->tire_parameters->RBY1 * cos(atan(car_parameters_->tire_parameters->RBY2 * (internal_vals.alpha_star - car_parameters_->tire_parameters->RBY3))) * car_parameters_->tire_parameters->LYKA;
  // (4.E60)
  double Gyk0 = cos (car_parameters_->tire_parameters->RCY1 * atan(Byk * SHyk - Eyk * (Byk * SHyk - atan(Byk * SHyk))));
  // (4.E59)
  double Gyk = cos(car_parameters_->tire_parameters->RCY1* atan(Byk * shifted_slip_ratio - Eyk * (Byk * shifted_slip_ratio))) / Gyk0;
  // (4.E58)
  return Fy0 * Gyk + SVyk;
}

// This function is never used (Im stupid and thought that the pure slip aligning moment was needed for the combined slip calculation) either way it is here if we need it later for some reason.
float MF6_2::calculatePureMoment(double Fy0 , double normal_load , double SHy , double SVy , double By, double Cy) const {
// MZ0_prime calculation
  // ------------------ at calculation
  // (4.E35)
  double SHt =  car_parameters_->tire_parameters->QHZ1 + car_parameters_->tire_parameters->QHZ2 * internal_vals.dfz + (car_parameters_->tire_parameters->QHZ3 + car_parameters_->tire_parameters->QHZ4 * internal_vals.dfz) * internal_vals.gamma_star;
  // (4.E34)
  double at = internal_vals.alpha_star + SHt;
  // ------------------ Bt calculation
  // (4.E40)
  double Bt = (car_parameters_->tire_parameters->QBZ1 + car_parameters_->tire_parameters->QBZ2 * internal_vals.dfz + car_parameters_->tire_parameters->QBZ3 * internal_vals.dfz * internal_vals.dfz) * (1 + car_parameters_->tire_parameters->QBZ4 * internal_vals.gamma_star + car_parameters_->tire_parameters->QBZ5 * abs(internal_vals.gamma_star)) * internal_vals.Kya / car_parameters_->tire_parameters->LMUY;
  // ------------------ Dt calculation
  // (4.91)
  double zeta5 = cos(atan(car_parameters_->tire_parameters->QDTP1 * car_parameters_->tire_parameters->effective_tire_r * internal_vals.phi)); 
  // (4.E42) -> expanded to account for pressure changes (dpi = 0 to ignore)
  double Dt0 = normal_load * (car_parameters_->tire_parameters->effective_tire_r / internal_vals.Fz0_prime) * (car_parameters_->tire_parameters->QDZ1 + car_parameters_->tire_parameters->QDZ2 * internal_vals.dfz) * (1- car_parameters_->tire_parameters->PPZ1) * car_parameters_->tire_parameters->LTR * sign(vehicle_model_state.velocities.velocity_x);
  // (4.E43)
  double Dt = Dt0 * (1 + car_parameters_->tire_parameters->QDZ3 * internal_vals.gamma_star + car_parameters_->tire_parameters->QDZ4 * (internal_vals.gamma_star * internal_vals.gamma_star)) * zeta5;
  // ------------------ Et calculation
  // (4.E44)
  double Et = (car_parameters_->tire_parameters->QEZ1 + car_parameters_->tire_parameters->QEZ2 * internal_vals.dfz + car_parameters_->tire_parameters->QEZ3 * internal_vals.dfz * internal_vals.dfz) * (1 + (car_parameters_->tire_parameters->QEZ4 * internal_vals.gamma_star + car_parameters_->tire_parameters->QEZ5 * internal_vals.gamma_star * internal_vals.gamma_star) * (2/M_PI) * atan(Bt * car_parameters_->tire_parameters->QCZ1 * at));
  Et = (Et < 1) ? Et : 1.0;  // Et cannot be over 1
  // ------------------
  // (4.E6)
  double alpha_prime = acos(vehicle_model_state.velocities.velocity_x / internal_vals.Vc_prime);
  // (4.E33)
  double t0 = Dt * cos(car_parameters_->tire_parameters->QCZ1 * atan(Bt * at - Et * (Bt * at - atan(Bt* at)))) * cos(alpha_prime);
  // (4.E32)
  double MZ0_prime = -t0 * Fy0;
// MZr0 calculation
  // (4.E47)
  double Dr = normal_load * car_parameters_->tire_parameters->effective_tire_r * ((car_parameters_->tire_parameters->QDZ6 + car_parameters_->tire_parameters->QDZ7 * internal_vals.dfz) * car_parameters_->tire_parameters->LRES * internal_vals.zeta2 + (car_parameters_->tire_parameters->QDZ8 + car_parameters_->tire_parameters->QDZ9 * internal_vals.dfz) * internal_vals.gamma_star * car_parameters_->tire_parameters->LKZC * internal_vals.zeta0) * cos(alpha_prime) * car_parameters_->tire_parameters->LMUY * sign(vehicle_model_state.velocities.velocity_x) + internal_vals.zeta8 -1;
  // (4.E45)
  double Br = car_parameters_->tire_parameters->QBZ10 * By * Cy;
  // (4.E38)
  double SHf = SHy + SVy / (internal_vals.Kya + internal_vals.epsilon * sign(internal_vals.Kya)); 
  // (4.E37)
  double ar = internal_vals.alpha_star + SHf;
  // (4.E36)
  double MZr0 = Dr  * cos (internal_vals.zeta7 * atan(Br * ar))
  // (4.E31)
  return MZ0_prime + MZr0;
}

float MF6_2::calculateCombinedMoment(double MZ0 , double Fx, double Fy, double slip_ratio , double SHy , double SVy , double By, double Cy) const {
// MZ_prime calculation
  // ------------------ at_eq calculation
  // (4.E35)
  double SHt =  car_parameters_->tire_parameters->QHZ1 + car_parameters_->tire_parameters->QHZ2 * internal_vals.dfz + (car_parameters_->tire_parameters->QHZ3 + car_parameters_->tire_parameters->QHZ4 * internal_vals.dfz) * internal_vals.gamma_star;
  // (4.E34)
  double at = internal_vals.alpha_star + SHt;
  // (4.E77)
  double at_eq = sqrt(at * at + (internal_vals.kxk / interval_vals.Kya_prime) * slip_ratio * slip_ratio * sign(at));
  // ------------------ Bt calculation
  // (4.E40)
  double Bt = (car_parameters_->tire_parameters->QBZ1 + car_parameters_->tire_parameters->QBZ2 * internal_vals.dfz + car_parameters_->tire_parameters->QBZ3 * internal_vals.dfz * internal_vals.dfz) * (1 + car_parameters_->tire_parameters->QBZ4 * internal_vals.gamma_star + car_parameters_->tire_parameters->QBZ5 * abs(internal_vals.gamma_star)) * internal_vals.Kya / car_parameters_->tire_parameters->LMUY;
  // ------------------ Dt calculation
  // (4.91)
  double zeta5 = cos(atan(car_parameters_->tire_parameters->QDTP1 * car_parameters_->tire_parameters->effective_tire_r * internal_vals.phi)); 
  // (4.E42) -> expanded to account for pressure changes (dpi = 0 to ignore)
  double Dt0 = normal_load * (car_parameters_->tire_parameters->effective_tire_r / internal_vals.Fz0_prime) * (car_parameters_->tire_parameters->QDZ1 + car_parameters_->tire_parameters->QDZ2 * internal_vals.dfz) * (1- car_parameters_->tire_parameters->PPZ1) * car_parameters_->tire_parameters->LTR * sign(vehicle_model_state.velocities.velocity_x);
  // (4.E43)
  double Dt = Dt0 * (1 + car_parameters_->tire_parameters->QDZ3 * internal_vals.gamma_star + car_parameters_->tire_parameters->QDZ4 * (internal_vals.gamma_star * internal_vals.gamma_star)) * zeta5;
  // ------------------ Et calculation
  // (4.E44)
  double Et = (car_parameters_->tire_parameters->QEZ1 + car_parameters_->tire_parameters->QEZ2 * internal_vals.dfz + car_parameters_->tire_parameters->QEZ3 * internal_vals.dfz * internal_vals.dfz) * (1 + (car_parameters_->tire_parameters->QEZ4 * internal_vals.gamma_star + car_parameters_->tire_parameters->QEZ5 * internal_vals.gamma_star * internal_vals.gamma_star) * (2/M_PI) * atan(Bt * car_parameters_->tire_parameters->QCZ1 * at));
  Et = (Et < 1) ? Et : 1.0;  // Et cannot be over 1
  // ------------------ t calculation
  // (4.E73)
  double t = Dt * cos(car_parameters_->tire_parameters->QCZ1 * atan(Bt * at_eq - Et * (Bt * at_eq - atan(Bt * at_eq)))) * cos(alpha_prime);
  // (4.E74)
  double Fy_prime = Fy - internal_vals.SVyk;
  // (4.E72)
  double MZ_prime = -t * Fy_prime;
// MZr calculation
  // (4.E47)
  double Dr = normal_load * car_parameters_->tire_parameters->effective_tire_r * ((car_parameters_->tire_parameters->QDZ6 + car_parameters_->tire_parameters->QDZ7 * internal_vals.dfz) * car_parameters_->tire_parameters->LRES * internal_vals.zeta2 + (car_parameters_->tire_parameters->QDZ8 + car_parameters_->tire_parameters->QDZ9 * internal_vals.dfz) * internal_vals.gamma_star * car_parameters_->tire_parameters->LKZC * internal_vals.zeta0) * cos(alpha_prime) * car_parameters_->tire_parameters->LMUY * sign(vehicle_model_state.velocities.velocity_x) + internal_vals.zeta8 -1;
  // (4.E45)
  double Br = car_parameters_->tire_parameters->QBZ10 * By * Cy;
  // (4.E38)
  double SHf = SHy + SVy / (internal_vals.Kya + internal_vals.epsilon * sign(internal_vals.Kya)); 
  // (4.E37)
  double ar = internal_vals.alpha_star + SHf;
  // (4.E78)
  double ar_eq = sqrt(ar * ar + (internal_vals.kxk / interval_vals.Kya_prime) * slip_ratio * slip_ratio * sign(ar));
  // (4.E75)
  double MZr = Dr * cos(internal_vals.zeta7 * atan(Br * ar_eq));
  // (4.E71)
// s calculation
  // (4.E76)
  double s = car_parameters_->tire_parameters->effective_tire_r * (car_parameters_->tire_parameters->SSZ1 + car_parameters_->tire_parameters->SSZ2 * (Fy / internal_vals.Fz0_prime) + (car_parameters_->tire_parameters->SSZ3 + car_parameters_->tire_parameters->SSZ4 * internal_vals.dfz) * internal_vals.gamma_star) * car_parameters_->tire_parameters->LS; 
  return MZ_prime + MZr + s * Fx;
}
/*
------------------------------------------------
               VD Implementation
------------------------------------------------
*/

/*
  Relevant parameters:
    - Q_FZ2 - Quadratic term in load vs defelction
    - epsilong -> camber reduction factor
    - gamma -> camberAngle
    - LMUX -> Scale factor of Fx peak friction coeficient
    - LMUY -> Scale factor of Fx peak friction coeficient
    - PDY1,2,3
    - PPY3,4
*/

/*
Relevant equations for Fx and Fy:

      -------------------------------------------------------------------
            D calculation:

            ux = (PDX1 + PDX2.*dfz).*(1 + PPX3.*dpi + PPX4.*dpi.^2).*(1 -PDX3.*gamma.^2).*LMUX_star;
% (4.E13) mux(Fz==0) = 0; % Zero Fz correction Dx = mux.*Fz.*zeta1; % (> 0) (4.E12)
      -----------------------------------------------------------------------


      -----------------------------------------------------------------------
        C calculation:
                    Cx = PCX1.*LCX; % (> 0) (4.E11)
      --------------------------------------------------------------------------

      ----------------------------------------------------------------------------
        B calculation:
                      Kxk = Fz.*(PKX1 + PKX2.*dfz).*exp(PKX3.*dfz).*(1 + PPX1.*dpi +
PPX2.*dpi.^2).*LKX;  % (= BxCxDx = dFxo./dkx at kappax = 0) (= Cfk) (4.E15)

            signDx = sign(Dx);
            signDx(signDx == 0) = 1; % If [Dx = 0] then [sign(0) = 0]. This is done to avoid [Kxk /
0 = NaN] in Eqn 4.E16

            Bx = Kxk./(Cx.*Dx + epsilonx.*signDx); % (4.E16) [sign(Dx) term explained on page 177]
      ---------------------------------------------------------------------------


      -----------------------------------------------
        E calculation:
          Ex = (PEX1 + PEX2.*dfz + PEX3.*dfz.^2).*(1 - PEX4.*sign(kappax)).*LEX; % (<=1) (4.E14)
      ------------------------------------------------

      ---------------------------------
        Fx:
                    Fx0 = Dx.*sin(Cx.*atan(Bx.*kappax-Ex.*(Bx.*kappax-atan(Bx.*kappax))))+SVx; %
(4.E9)
      -----------------------------------
*/

/*
  Relevant equations for Fy:

      --------------------------
        C calculation:
                      Cy = PCY1.*LCY; % (> 0) (4.E21)
      --------------------------

      ---------------------------
        D calculation:
                      muy = (PDY1 + PDY2 .* dfz).*(1 + PPY3.*dpi + PPY4 .*dpi.^2).*(1 -
  PDY3.*gamma_star.^2).*LMUY_star; % (4.E23) Dy = muy.*Fz.*zeta2; % (4.E22)
      ------------------------------

      ------------------------------
        E calculation:
                      signAlphaY = sign(alphay);
            signAlphaY(signAlphaY == 0) = 1;
            Ey = (PEY1 + PEY2.*dfz).*(1 + PEY5.*gamma_star.^2 - (PEY3 +
  PEY4.*gamma_star).*signAlphaY).*LEY; % (<=1)(4.E24)
      -----------------------------

      -----------------------------
        B calculation:
                      signDy = sign(Dy);
            signDy(signDy == 0) = 1; % If [Dy = 0] then [sign(0) = 0]. This is done to avoid [Kya /
  0 = NaN] in Eqn 4.E26

            By = Kya./(Cy.*Dy + epsilony.*signDy); % (4.E26) [sign(Dy) term explained on page 177]
      ------------------------------


      ---------------------
      Fy:
                    Fy0 = Dy .* sin(Cy.*atan(By.*alphay-Ey.*(By.*alphay - atan(By.*alphay))))+ SVy;
  % (4.E19)





*/