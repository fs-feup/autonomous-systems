#include "VehicleModel/VehicleModelBicycle.hpp"

#include <cmath>

#include "VehicleModel/VehicleModelInterface.hpp"
#include "transform.hpp"

VehicleModelBicycle::VehicleModelBicycle()
    : position(Eigen::Vector3d::Zero()),
      orientation(Eigen::Vector3d::Zero()),
      velocity(Eigen::Vector3d::Zero()),
      angularVelocity(Eigen::Vector3d::Zero()),
      acceleration(Eigen::Vector3d::Zero()) {}

bool VehicleModelBicycle::readConfig(ConfigElement& config) {
  auto configModel = config["simple_bicycle_model"];

  // Read kinematics parameters
  auto kinematics = configModel["kinematics"];
  kinematics.getElement<double>(&lf, "lf");
  kinematics.getElement<double>(&lr, "lr");
  kinematics.getElement<double>(&sf, "sf");
  kinematics.getElement<double>(&sr, "sr");

  // Read tire model parameters
  auto tire = configModel["tire"];
  tire.getElement<double>(&tireModel.Blat, "Blat");
  tire.getElement<double>(&tireModel.Clat, "Clat");
  tire.getElement<double>(&tireModel.Dlat, "Dlat");
  tire.getElement<double>(&tireModel.Elat, "Elat");

  // Read aerodynamic parameters
  auto aero = configModel["aero"];
  aero.getElement<double>(&aeroModel.cla, "cla");
  aero.getElement<double>(&aeroModel.cda, "cda");
  aero.getElement<double>(&aeroModel.aeroArea, "aeroArea");

  // Read general vehicle parameters
  configModel.getElement<double>(&m, "m");
  configModel.getElement<double>(&Izz, "Izz");
  configModel.getElement<double>(&powertrainModel.wheelRadius, "wheelRadius");
  configModel.getElement<double>(&powertrainModel.gearRatio, "gearRatio");
  configModel.getElement<double>(&steeringModel.innerSteeringRatio, "innerSteeringRatio");
  configModel.getElement<double>(&steeringModel.outerSteeringRatio, "outerSteeringRatio");
  configModel.getElement<double>(&powertrainModel.nominalVoltageTS, "nominalVoltageTS");
  configModel.getElement<double>(&powertrainModel.powerGroundForce, "powerGroundForce");

  return true;
}

Eigen::Vector3d VehicleModelBicycle::getPosition() { return position; }

Eigen::Vector3d VehicleModelBicycle::getOrientation() { return orientation; }

Eigen::Vector3d VehicleModelBicycle::getVelocity() { return velocity; }

Eigen::Vector3d VehicleModelBicycle::getAcceleration() { return acceleration; }

Eigen::Vector3d VehicleModelBicycle::getAngularVelocity() { return angularVelocity; }

Eigen::Vector3d VehicleModelBicycle::getAngularAcceleration() { return angularAcceleration; }

Wheels VehicleModelBicycle::getSteeringAngles() { return steeringAngles; }

double VehicleModelBicycle::getSteeringWheelAngle() {
  return steeringModel.calculateSteeringWheelAngle(steeringAngles);
}

double VehicleModelBicycle::getVoltageTS() { return powertrainModel.nominalVoltageTS; }

double VehicleModelBicycle::getCurrentTS() {
  return powertrainModel.calculateCurrent(torques, wheelspeeds, powertrainModel.nominalVoltageTS);
}

Wheels VehicleModelBicycle::getWheelspeeds() { return wheelspeeds; }

Wheels VehicleModelBicycle::getWheelOrientations() { return wheelOrientations; }

Wheels VehicleModelBicycle::getTorques() { return torques; }

void VehicleModelBicycle::setTorques(Wheels in) { torques = in; }

void VehicleModelBicycle::setRpmSetpoints(Wheels in) { rpmSetpoints = in; }

void VehicleModelBicycle::setMaxTorques(Wheels in) { maxTorques = in; }

void VehicleModelBicycle::setMinTorques(Wheels in) { minTorques = in; }

void VehicleModelBicycle::setSteeringSetpointFront(double in) {
  steeringModel.calculateSteeringAngles(in, steeringAngles);
}

void VehicleModelBicycle::setSteeringSetpointRear(double in) {
  // Rear steering not implemented
}

void VehicleModelBicycle::setThrottle(double in) { throttleActuation = std::clamp(in, -1.0, 1.0); }

void VehicleModelBicycle::setPowerGroundSetpoint(double in) {
  powerGroundSetpoint = std::clamp(in, 0.0, 1.0);
}

void VehicleModelBicycle::setPosition(Eigen::Vector3d newPosition) { position = newPosition; }

void VehicleModelBicycle::setOrientation(Eigen::Vector3d newOrientation) {
  orientation = newOrientation;
}

double VehicleModelBicycle::processSlipAngleLat(double alpha_input, double Fz) {
  return tireModel.calculateLateralForce(alpha_input, Fz);
}

void VehicleModelBicycle::AerodynamicsModel::calculateForces(double velocityX, double& downforce,
                                                             double& drag) const {
  double velocitySquared = velocityX * velocityX;
  downforce = 0.5 * AIR_DENSITY * aeroArea * cla * velocitySquared;
  double dragMagnitude = 0.5 * AIR_DENSITY * aeroArea * cda * velocitySquared;
  drag = (velocityX > 0) ? -dragMagnitude : dragMagnitude;
}

int VehicleModelBicycle::TireModel::sign(double value) const {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 1;  // Default to 1 for zero
}

double VehicleModelBicycle::TireModel::calculateLateralForce(double slipAngle,
                                                             double normalForce) const {
  double dpi = (p_input - NOMPRES) / NOMPRES;
  double Fz_0_prime = LFZO * FNOM;
  double Kya = PKY1 * Fz_0_prime * (1 + PPY1 * dpi) * (1 - PKY3 * abs(y_input)) *
               sin(PKY4 * atan((normalForce / Fz_0_prime) /
                               ((PKY2 + PKY5 * y_input * y_input) * (1 + PPY2 * dpi)))) *
               zeta_3 * LKY;
  double dfz = (normalForce - Fz_0_prime) / Fz_0_prime;
  double Kyg0 = normalForce * (PKY6 + PKY7 * dfz) * (1 + PPY5 * dpi) * LKYC;
  double Vs_y = tan(slipAngle) * abs(V_cx);
  double Vs_x = -slip_ratio * abs(V_cx);
  double Vs = sqrt(Vs_x * Vs_x + Vs_y * Vs_y);
  double V0 = LONGVL;
  double LMUY_star = LMUY / (1.0 + LMUV * (Vs / V0));
  double SVyg = normalForce * (PVY3 + PVY4 * dfz) * y_input * LKYC * LMUY_star * zeta_2;
  double SHy = (PHY1 + PHY2 * dfz) +
               ((Kyg0 * y_input - SVyg) / (Kya + eps_k * sign(Kya))) * zeta_0 + zeta_4 - 1.0;
  double alpha_y = slipAngle + SHy;
  double Ey = (PEY1 + PEY2 * dfz) *
              (1 + PEY5 * y_input * y_input - (PEY3 + PEY4 * y_input) * sign(alpha_y)) * LEY;
  double Cy = PCY1 * LCY;
  double mu_y = (PDY1 + PDY2 * dfz) * (1.0 + PPY3 * dpi + PPY4 * dpi * dpi) *
                (1.0 - PDY3 * y_input * y_input) * LMUY_star;
  double Dy = mu_y * normalForce * zeta_2;
  double By = Kya / (Cy * Dy + eps_y * sign(Dy));
  double DVyk = mu_y * normalForce * (RVY1 + RVY2 * dfz + RVY3 * y_input) *
                cos(atan(RVY4 * slipAngle)) * zeta_2;
  double SVyk = DVyk * sin(RVY5 * atan(RVY6 * slip_ratio)) * LVYKA;
  double Eyk = REY1 + REY2 * dfz;
  double SHyk = RHY1 + RHY2 * dfz;
  double ks = slip_ratio + SHyk;
  double Byk = (RBY1 + RBY4 * y_input * y_input) * cos(atan(RBY2 * (slipAngle - RBY3))) * LYKA;
  double Cyk = RCY1;
  double Gyk_0 = cos(Cyk * atan(Byk * SHyk - Eyk * (Byk * SHyk - atan(Byk * SHyk))));
  double Gyk = (cos(Cyk * atan(Byk * ks - Eyk * (Byk * ks - atan(Byk * ks)))) / Gyk_0);
  double SVy = normalForce * (PVY1 + PVY2 * dfz) * LVY * LMUY_star * zeta_2 + SVyg;
  double Fy_0 = Dy * sin(Cy * atan(By * alpha_y - Ey * (By * alpha_y - atan(By * alpha_y)))) + SVy;
  double Fy = Gyk * Fy_0 + SVyk;

  return Fy;
}

void VehicleModelBicycle::PowertrainModel::calculateWheelTorques(double throttleInput,
                                                                 Wheels& torques) const {
  // Simple rear-wheel drive implementation
  torques.FL = 0;
  torques.FR = 0;
  torques.RL = throttleInput * MAX_TORQUE * 0.5;
  torques.RR = throttleInput * MAX_TORQUE * 0.5;
}

double VehicleModelBicycle::PowertrainModel::calculateEfficiency(const Wheels& torques) const {
  // Simple efficiency model based on rear wheel torques
  return 0.002333 * (torques.RL + torques.RR) + 0.594;
}

double VehicleModelBicycle::PowertrainModel::calculateCurrent(const Wheels& torques,
                                                              const Wheels& wheelspeeds,
                                                              double voltage) const {
  constexpr double powerCoeff = 1.0 / 9.55;

  // Calculate power for each wheel
  double powerFL = torques.FL * wheelspeeds.FL * powerCoeff;
  double powerFR = torques.FR * wheelspeeds.FR * powerCoeff;
  double powerRL = torques.RL * wheelspeeds.RL * powerCoeff;
  double powerRR = torques.RR * wheelspeeds.RR * powerCoeff;

  // Calculate total power and return current
  double totalPower = (powerFL + powerFR + powerRL + powerRR) / calculateEfficiency(torques);
  return (totalPower / voltage);
}

void VehicleModelBicycle::SteeringModel::calculateSteeringAngles(double steeringInput,
                                                                 Wheels& steeringAngles) const {
  // Apply Ackermann steering geometry
  double avgRatio = 0.5 * (innerSteeringRatio + outerSteeringRatio);

  if (steeringInput > 0) {
    steeringAngles.FL = innerSteeringRatio * steeringInput / avgRatio;
    steeringAngles.FR = outerSteeringRatio * steeringInput / avgRatio;
  } else {
    steeringAngles.FL = outerSteeringRatio * steeringInput / avgRatio;
    steeringAngles.FR = innerSteeringRatio * steeringInput / avgRatio;
  }

  // Rear wheels have zero steering angle
  steeringAngles.RL = 0.0;
  steeringAngles.RR = 0.0;
}

double VehicleModelBicycle::SteeringModel::calculateSteeringWheelAngle(
    const Wheels& steeringAngles) const {
  // Calculate steering wheel angle based on the front left steering angle and appropriate ratio
  return (steeringAngles.FL > 0) ? steeringAngles.FL / innerSteeringRatio
                                 : steeringAngles.FL / outerSteeringRatio;
}

void VehicleModelBicycle::calculateNormalForces(double& Fz_Front, double& Fz_Rear) const {
  double l = lr + lf;

  // Calculate downforce from aerodynamics
  double downforce = 0.0;
  double drag = 0.0;
  aeroModel.calculateForces(velocity.x(), downforce, drag);

  // Add ground effect if requested
  downforce += powerGroundSetpoint * powertrainModel.powerGroundForce;

  // Calculate normal forces on axles with weight distribution and aero effects
  Fz_Front = std::max(0.0, (m * GRAVITY + downforce) * 0.5 * lr / l);
  Fz_Rear = std::max(0.0, (m * GRAVITY + downforce) * 0.5 * lf / l);
}

void VehicleModelBicycle::calculateSlipAngles(double& kappaFront, double& kappaRear) const {
  constexpr double eps = 0.00001;  // Prevent division by zero
  
  /**
   * It seems like this isn't needed anymore with the approach of applying friction to
   * the opposite of the velocity vector instead of Vx only. Drag also applied opposite of Vx
   * instead of simply adding the negative value.
   */
  // bool stillstand = (velocity.norm() < VELOCITY_THRESHOLD) &&
  //                   (std::abs(angularVelocity.z()) < ANGULAR_VELOCITY_THRESHOLD);

  // Calculate wheel positions relative to CoG
  Eigen::Vector3d rFront(lf, 0.0, 0.0);
  Eigen::Vector3d rRear(-lr, 0.0, 0.0);

  // Calculate wheel velocities
  Eigen::Vector3d vFront = velocity + angularVelocity.cross(rFront);
  Eigen::Vector3d vRear = velocity + angularVelocity.cross(rRear);

  // Calculate average front steering angle
  double steeringFront = 0.5 * (steeringAngles.FL + steeringAngles.FR);

  // Calculate slip angles for front and rear
  kappaFront = std::atan2(vFront.y(), std::max(std::abs(vFront.x()), eps)) - steeringFront;
  kappaRear = std::atan2(vRear.y(), std::max(std::abs(vRear.x()), eps));

  // if (stillstand) {
  //   kappaFront = 0.0;
  //   kappaRear = 0.0;
  // }
}

// New helper function to calculate wheel positions and velocities
void VehicleModelBicycle::calculateWheelGeometry(double& steeringFront, Eigen::Vector3d& rFL,
                                                 Eigen::Vector3d& rFR, Eigen::Vector3d& rRL,
                                                 Eigen::Vector3d& rRR, Eigen::Vector3d& rFront,
                                                 Eigen::Vector3d& rRear, Eigen::Vector3d& vFL,
                                                 Eigen::Vector3d& vFR, Eigen::Vector3d& vRL,
                                                 Eigen::Vector3d& vRR) const {
  // Calculate average front steering angle
  steeringFront = 0.5 * (steeringAngles.FL + steeringAngles.FR);

  // Calculate wheel positions relative to CoG
  rFL = Eigen::Vector3d(lf, 0.5 * sf, 0.0);
  rFR = Eigen::Vector3d(lf, -0.5 * sf, 0.0);
  rRL = Eigen::Vector3d(-lr, 0.5 * sr, 0.0);
  rRR = Eigen::Vector3d(-lr, -0.5 * sr, 0.0);
  rFront = Eigen::Vector3d(lf, 0.0, 0.0);
  rRear = Eigen::Vector3d(-lr, 0.0, 0.0);

  // Calculate wheel velocities
  vFL = velocity + angularVelocity.cross(rFL);
  vFR = velocity + angularVelocity.cross(rFR);
  vRL = velocity + angularVelocity.cross(rRL);
  vRR = velocity + angularVelocity.cross(rRR);
}

// New helper function to calculate longitudinal forces
void VehicleModelBicycle::calculateLongitudinalForces(double& Fx_FL, double& Fx_FR, double& Fx_RL,
                                                      double& Fx_RR) const {
  // Calculate powertrain efficiency
  double powertrainEfficiency = powertrainModel.calculateEfficiency(torques);

  // Calculate longitudinal forces on wheels
  Fx_FL = 0;  // Front wheel drive not enabled
  Fx_FR = 0;  // Front wheel drive not enabled
  Fx_RL =
      (powertrainModel.gearRatio * torques.RL / powertrainModel.wheelRadius) * powertrainEfficiency;
  Fx_RR =
      (powertrainModel.gearRatio * torques.RR / powertrainModel.wheelRadius) * powertrainEfficiency;

  // Apply forces only when torque is significant or vehicle is moving
  Fx_FL *= (torques.FL > TORQUE_THRESHOLD || velocity.x() > VELOCITY_MIN_THRESHOLD) ? 1.0 : 0.0;
  Fx_FR *= (torques.FR > TORQUE_THRESHOLD || velocity.x() > VELOCITY_MIN_THRESHOLD) ? 1.0 : 0.0;
  Fx_RL *= (torques.RL > TORQUE_THRESHOLD || velocity.x() > VELOCITY_MIN_THRESHOLD) ? 1.0 : 0.0;
  Fx_RR *= (torques.RR > TORQUE_THRESHOLD || velocity.x() > VELOCITY_MIN_THRESHOLD) ? 1.0 : 0.0;
}

// New helper function to calculate accelerations
Eigen::Vector3d VehicleModelBicycle::calculateAccelerations(double steeringFront, double Fx_FL,
                                                            double Fx_FR, double Fx_RL,
                                                            double Fx_RR, double Fy_Front,
                                                            double Fy_Rear, double drag,
                                                            const Eigen::Vector3d& friction) const {
  // Calculate longitudinal acceleration from tire forces
  double axTires = (std::cos(steeringAngles.FL) * Fx_FL + std::cos(steeringAngles.FR) * Fx_FR +
                    Fx_RL + Fx_RR - std::sin(steeringFront) * Fy_Front) /
                   m;

  // Apply aerodynamic drag and friction to get total acceleration
  double axModel = axTires + drag / m + friction.x() / m;


  // Calculate lateral acceleration from tire forces
  double ayTires = (std::sin(steeringAngles.FL) * Fx_FL + std::sin(steeringAngles.FR) * Fx_FR +
                    std::cos(steeringFront) * Fy_Front + Fy_Rear) /
                   m;
  double ayModel = ayTires + friction.y() / m;


  // Calculate yaw moment contributions
  double rdotFx =
      0.5 * sf * (-Fx_FL * std::cos(steeringAngles.FL) + Fx_FR * std::cos(steeringAngles.FR)) +
      lf * (Fx_FL * std::sin(steeringAngles.FL) + Fx_FR * std::sin(steeringAngles.FR)) +
      0.5 * sr * (Fx_RR - Fx_RL);

  double rdotFy = lf * (Fy_Front * std::cos(steeringFront)) - lr * (Fy_Rear);

  // Calculate yaw acceleration
  double rdot = (rdotFx + rdotFy) / Izz;

  return Eigen::Vector3d(axModel, ayModel, rdot);
}

// New helper function to update wheel speeds
void VehicleModelBicycle::updateWheelSpeeds(const Eigen::Vector3d& vFL, const Eigen::Vector3d& vFR,
                                            const Eigen::Vector3d& vRL,
                                            const Eigen::Vector3d& vRR) {
  // Conversion factor from RPM to m/s
  double rpm2ms = powertrainModel.wheelRadius * TWO_PI / 60;

  // Update wheel speeds
  wheelspeeds.FL = vFL.x() / rpm2ms;
  wheelspeeds.FR = vFR.x() / rpm2ms;
  wheelspeeds.RL = vRL.x() / rpm2ms;
  wheelspeeds.RR = vRR.x() / rpm2ms;
}

// Refactored getDynamicStates using the new helper functions
Eigen::Vector3d VehicleModelBicycle::getDynamicStates(double dt) {
  // Calculate wheel positions and velocities
  double steeringFront;
  Eigen::Vector3d rFL, rFR, rRL, rRR, rFront, rRear;
  Eigen::Vector3d vFL, vFR, vRL, vRR;
  calculateWheelGeometry(steeringFront, rFL, rFR, rRL, rRR, rFront, rRear, vFL, vFR, vRL, vRR);

  // Calculate normal forces on axles
  double Fz_Front, Fz_Rear;
  calculateNormalForces(Fz_Front, Fz_Rear);

  // Calculate aero drag
  double downforce, drag;
  aeroModel.calculateForces(velocity.x(), downforce, drag);

  // Calculate rolling resistance (applied opposite to velocity direction)
  double frict = (Fz_Front + Fz_Rear) * ROLLING_RESISTANCE_COEFF;
  Eigen::Vector3d friction;

  if (velocity.norm() < 0.01) {
      friction = Eigen::Vector3d::Zero();
  } else {
      Eigen::Vector3d velDir = -velocity.normalized();
      friction = frict * velDir;
  }


  // Calculate longitudinal forces
  double Fx_FL, Fx_FR, Fx_RL, Fx_RR;
  calculateLongitudinalForces(Fx_FL, Fx_FR, Fx_RL, Fx_RR);

  // Calculate slip angles
  double kappaFront, kappaRear;
  calculateSlipAngles(kappaFront, kappaRear);

  // Calculate lateral forces using tire model
  double Fy_Front = processSlipAngleLat(kappaFront, Fz_Front / 2) * 2;
  double Fy_Rear = processSlipAngleLat(kappaRear, Fz_Rear / 2) * 2;

  // Update wheel speeds
  updateWheelSpeeds(vFL, vFR, vRL, vRR);

  RCLCPP_DEBUG(rclcpp::get_logger("a"),
      "Vx: %f, Vy: %f, W: %f, KappaF: %f, KappaR: %f, Fy_Front: %f, Fy_Rear: %f, drag: %f, friction: %f", velocity.x(),
      velocity.y(), angularVelocity.z(), kappaFront, kappaRear, Fy_Front, Fy_Rear, drag, friction.x());

  // Calculate accelerations
  return calculateAccelerations(steeringFront, Fx_FL, Fx_FR, Fx_RL, Fx_RR, Fy_Front, Fy_Rear, drag,
                                friction);
}

void VehicleModelBicycle::forwardIntegrate(double dt) {
  // Update position based on velocity and orientation
  Eigen::AngleAxisd yawAngle(orientation.z(), Eigen::Vector3d::UnitZ());
  position += (yawAngle.matrix() * velocity) * dt;

  // Convert throttle to wheel torques (using PowertrainModel)
  powertrainModel.calculateWheelTorques(throttleActuation, torques);

  // Calculate dynamic states
  Eigen::Vector3d dynamicStates = getDynamicStates(dt);

  // Update orientation based on angular velocity
  orientation.z() += dt * angularVelocity.z();

  // Update acceleration from dynamic states
  acceleration = Eigen::Vector3d(dynamicStates.x(), dynamicStates.y(), 0.0);

  // Update angular velocity and acceleration
  angularVelocity.z() += dynamicStates.z() * dt;
  angularAcceleration.z() = dynamicStates.z();

  // Update velocity with acceleration and angular velocity effects
  velocity += dt * (acceleration - angularVelocity.cross(velocity));

  // Update wheel orientations
  const double wheelRotationFactor = TWO_PI / (60.0 * powertrainModel.gearRatio);

  wheelOrientations.FL =
      std::fmod(wheelOrientations.FL + wheelspeeds.FL * dt * wheelRotationFactor, TWO_PI);
  wheelOrientations.FR =
      std::fmod(wheelOrientations.FR + wheelspeeds.FR * dt * wheelRotationFactor, TWO_PI);
  wheelOrientations.RL =
      std::fmod(wheelOrientations.RL + wheelspeeds.RL * dt * wheelRotationFactor, TWO_PI);
  wheelOrientations.RR =
      std::fmod(wheelOrientations.RR + wheelspeeds.RR * dt * wheelRotationFactor, TWO_PI);
}

std::array<Eigen::Vector3d, 4> VehicleModelBicycle::getWheelPositions() {
  // Transform wheel positions from vehicle coordinates to world coordinates
  auto rotMat = eulerAnglesToRotMat(orientation).transpose();

  Eigen::Vector3d FL = rotMat * Eigen::Vector3d(lf, sf * 0.5, 0.0) + position;
  Eigen::Vector3d FR = rotMat * Eigen::Vector3d(lf, -sf * 0.5, 0.0) + position;
  Eigen::Vector3d RL = rotMat * Eigen::Vector3d(-lr, sr * 0.5, 0.0) + position;
  Eigen::Vector3d RR = rotMat * Eigen::Vector3d(-lr, -sr * 0.5, 0.0) + position;

  return {FL, FR, RL, RR};
}