#include "motion_lib/tire_model/pacejka_bombado.hpp"

float PacejkaBombado::calculateSlipAngle() const {
  double v_x = vehicle_model_state.velocities.velocity_x;
  if (v_x == 0.0) v_x = 0.1;
  float numerator = vehicle_model_state.velocities.velocity_y +
                    (internal_vals.distance_to_CG * vehicle_model_state.yaw_rate);
  return atan2(
      numerator,
      v_x);  // atan2 permite cobrir todos os ranges de angulo e não apenas de -pi/2 até pi/2
}

float PacejkaBombado::calculateSlipRatio() const {
  double v_x = vehicle_model_state.velocities.velocity_x;
  if (v_x == 0.0) v_x = 0.1;
  float numerator =
      (car_parameters_->tire_parameters->effective_tire_r * vehicle_model_state.angular_speed) -
      v_x;
  return numerator / v_x;
}

std::pair<double, double> PacejkaBombado::tire_forces(double slip_angle, double slip_ratio,
                                                      double vertical_load) const {
  float slip_a = calculateSlipAngle();
  float slip_r = calculateSlipRatio();
  float Dy = calculateDy();
  float Cy = calculateCy();
  float By = calculateBy(Dy, Cy);
  float Ey = calculateEy(slip_a);
  float Dx = calculateDx();
  float Cx = calculateCx();
  float Bx = calculateBx(Dx, Cx);
  float Ex = calculateEx(slip_r);
  float SVx = calculateSVx();
  float SVy = calculateSVy();

  double Fx = calculatePacejka(Bx, Cx, Dx, Ex, slip_r, SVx);
  double Fy = calculatePacejka(By, Cy, Dy, Ey, slip_a, SVy);

  (void)slip_angle;     // to avoid unused variable warning
  (void)slip_ratio;     // to avoid unused variable warning
  (void)vertical_load;  // to avoid unused variable warning

  // After the definition of the input of tire models this needs to be deleted
  return {Fx, Fy};
}

float PacejkaBombado::calculatePacejka(float B, float C, float D, float E, float slip,
                                       float SV) const {
  return D * sin(C * atan(B * slip - E * (B * slip - atan(B * slip)))) + SV;
}

/*
---------------------------
  D calculation:
    muy = (PDY1 + PDY2 .* dfz).*(1 + PPY3.*dpi + PPY4 .*dpi.^2).*(1
-PDY3.*gamma_star.^2).*LMUY_star; % (4.E23)
    Dy = muy.*Fz.*zeta2; % (4.E22)
------------------------------
*/

bool PacejkaBombado::calculateTireState(float slip_angle, float slip_ratio) {
  internal_vals.Fz0_prime = car_parameters_->tire_parameters->LFZO * vehicle_model_state.forces.z;
  internal_vals.dfz =
      (vehicle_model_state.forces.z - internal_vals.Fz0_prime) / internal_vals.Fz0_prime;
  internal_vals.epsilong = car_parameters_->tire_parameters->PECP1 *
                           (1 + car_parameters_->tire_parameters->PECP2 * internal_vals.dfz);
  internal_vals.Vc_prime =
      sqrt(vehicle_model_state.velocities.velocity_x * vehicle_model_state.velocities.velocity_x +
           vehicle_model_state.velocities.velocity_y * vehicle_model_state.velocities.velocity_y);
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
          atan((vehicle_model_state.forces.z / internal_vals.Fz0_prime) /
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
  double Bxp = car_parameters_->tire_parameters->PDXP1 *
               (1 + car_parameters_->tire_parameters->PDXP2 * internal_vals.dfz) *
               cos(atan(car_parameters_->tire_parameters->PDXP3 * slip_ratio));
  internal_vals.zeta1 =
      cos(atan(Bxp * car_parameters_->tire_parameters->effective_tire_r * internal_vals.phi));
  internal_vals.LMUY_prime =
      car_parameters_->tire_parameters->Amu * car_parameters_->tire_parameters->LMUY /
      (1 + (car_parameters_->tire_parameters->Amu - 1) * car_parameters_->tire_parameters->LMUY);
  internal_vals.SVyg = vehicle_model_state.forces.z *
                       (car_parameters_->tire_parameters->PVY3 +
                        car_parameters_->tire_parameters->PVY4 * internal_vals.dfz) *
                       internal_vals.gamma_star * car_parameters_->tire_parameters->LKYC *
                       internal_vals.LMUY_prime * internal_vals.zeta2;
  return true;
}

float PacejkaBombado::calculateDy() const {
  double muy = (car_parameters_->tire_parameters->PDY1 +
                car_parameters_->tire_parameters->PDY2 * internal_vals.dfz) *
               (1 + car_parameters_->tire_parameters->PPY3 * internal_vals.dpi +
                car_parameters_->tire_parameters->PPY4 * (internal_vals.dpi * internal_vals.dpi)) *
               (1 - car_parameters_->tire_parameters->PDY3 *
                        (internal_vals.gamma_star * internal_vals.gamma_star)) *
               car_parameters_->tire_parameters
                   ->LMUY;  // we can assume dpi is zero for now and ignore pressure variations

  return muy * vehicle_model_state.forces.z * internal_vals.zeta2;
}

float PacejkaBombado::calculateDx() const {
  if (vehicle_model_state.forces.z > 0) {
    double mux =
        (car_parameters_->tire_parameters->PDX1 +
         car_parameters_->tire_parameters->PDX2 * internal_vals.dfz) *
        (1 + car_parameters_->tire_parameters->PPX3 * internal_vals.dpi +
         car_parameters_->tire_parameters->PPX4 * (internal_vals.dpi * internal_vals.dpi)) *
        (1 - car_parameters_->tire_parameters->PDX3 *
                 (internal_vals.camber_angle * internal_vals.camber_angle)) *
        car_parameters_->tire_parameters->LMUX;
    return mux * vehicle_model_state.forces.z * internal_vals.zeta1;
  }
  return 0;
}

float PacejkaBombado::calculateCx() const {
  if (vehicle_model_state.forces.z > 0) {
    return car_parameters_->tire_parameters->PCX1 * car_parameters_->tire_parameters->LCX;
  }
  return 0;
}

float PacejkaBombado::calculateCy() const {
  if (vehicle_model_state.forces.z > 0) {
    return car_parameters_->tire_parameters->PCY1 * car_parameters_->tire_parameters->LCY;
  }
  return 0;
}

float PacejkaBombado::calculateBx(float Dx, float Cx) const {
  double Kxk = vehicle_model_state.forces.z *
               (car_parameters_->tire_parameters->PKX1 +
                car_parameters_->tire_parameters->PKX2 * internal_vals.dfz) *
               exp(car_parameters_->tire_parameters->PKX3 * internal_vals.dfz) *
               (1 + car_parameters_->tire_parameters->PPX1 * internal_vals.dpi +
                car_parameters_->tire_parameters->PPX2 * (internal_vals.dpi * internal_vals.dpi)) *
               car_parameters_->tire_parameters->LKX;
  return Kxk / (Cx * Dx + internal_vals.epsilon * sign(Dx));
}

float PacejkaBombado::calculateBy(float Dy, float Cy) const {
  return internal_vals.Kya / (Cy * Dy + internal_vals.epsilon * sign(Dy));
}

float PacejkaBombado::calculateEx(double slip_ratio) const {
  double SHx = (car_parameters_->tire_parameters->PHX1 +
                car_parameters_->tire_parameters->PHX2 * internal_vals.dfz) *
               car_parameters_->tire_parameters->LHX;
  double kappax = slip_ratio + SHx;
  double Ex = (car_parameters_->tire_parameters->PEX1 +
               car_parameters_->tire_parameters->PEX2 * internal_vals.dfz +
               car_parameters_->tire_parameters->PEX3 * (internal_vals.dfz * internal_vals.dfz)) *
              (1 - car_parameters_->tire_parameters->PEX4 * sign(kappax)) *
              car_parameters_->tire_parameters->LEX;
  return (Ex < 1) ? Ex : 1.0;  // E cannot be over 1
}

float PacejkaBombado::calculateEy(double slip_angle) const {
  // MF 6.1 and 6.2
  double Kyg0 = vehicle_model_state.forces.z *
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
                sin(car_parameters_->tire_parameters->PKY4 *
                    atan((vehicle_model_state.forces.z / internal_vals.Fz0_prime) /
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
  double Kya_prime = internal_vals.Kya + internal_vals.epsilon * sign(internal_vals.Kya);
  double zeta4 = 1 + SHyp - internal_vals.SVyg / Kya_prime;
  double SHy = (car_parameters_->tire_parameters->PHY1 +
                car_parameters_->tire_parameters->PHY2 * internal_vals.dfz) *
                   car_parameters_->tire_parameters->LHY +
               ((Kyg0 * internal_vals.gamma_star - internal_vals.SVyg) /
                (internal_vals.Kya + internal_vals.epsilon * sign(internal_vals.Kya))) +
               zeta4 - 1;
  double alpha_y = tan(slip_angle) + SHy;
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

float PacejkaBombado::calculateSVx() const {
  double LMUX_prime =
      car_parameters_->tire_parameters->Amu * car_parameters_->tire_parameters->LMUX /
      (1 + (car_parameters_->tire_parameters->Amu - 1) * car_parameters_->tire_parameters->LMUX);
  return vehicle_model_state.forces.z *
         (car_parameters_->tire_parameters->PVX1 +
          car_parameters_->tire_parameters->PVX2 * internal_vals.dfz) *
         car_parameters_->tire_parameters->LVX * LMUX_prime * internal_vals.zeta1;
}

float PacejkaBombado::calculateSVy() const {
  return vehicle_model_state.forces.z *
             (car_parameters_->tire_parameters->PVY1 +
              car_parameters_->tire_parameters->PVY2 * internal_vals.dfz) *
             car_parameters_->tire_parameters->LVY * internal_vals.LMUY_prime *
             internal_vals.zeta2 +
         internal_vals.SVyg;
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