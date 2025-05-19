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
        double avgRatio = 0.5 * (this->steeringModel.innerSteeringRatio + this->steeringModel.outerSteeringRatio);
        if (in > 0)
        {
            this->steeringAngles.FL = this->steeringModel.innerSteeringRatio * in;
            this->steeringAngles.FR = this->steeringModel.outerSteeringRatio * in;
        }
        else
        {
            this->steeringAngles.FL = this->steeringModel.outerSteeringRatio * in;
            this->steeringAngles.FR = this->steeringModel.innerSteeringRatio * in;
        }
        return;
}

void VehicleModelBicycle::setSteeringSetpointRear(double in) {
  // Rear steering not implemented
}

void VehicleModelBicycle::setThrottle(double in) { throttleActuation = 0.1 * std::clamp(in, -1.0, 1.0); }

void VehicleModelBicycle::setPowerGroundSetpoint(double in) {
  powerGroundSetpoint = std::clamp(in, 0.0, 1.0);
}

void VehicleModelBicycle::setPosition(Eigen::Vector3d newPosition) { position = newPosition; }

void VehicleModelBicycle::setOrientation(Eigen::Vector3d newOrientation) {
  orientation = newOrientation;
}

double VehicleModelBicycle::processSlipAngleLat(double alpha_input, double Fz = 0.0) {
  return std::sin(tireModel.Clat * std::atan(tireModel.Blat * alpha_input - tireModel.Elat * (tireModel.Blat * alpha_input - std::atan(tireModel.Blat * alpha_input))));
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
  double rdotFx
    = 0.5 * this->sf * (-Fx_FL * std::cos(this->steeringAngles.FL) + Fx_FR * std::cos(this->steeringAngles.FR))
    + this->lf * (Fx_FL * std::sin(this->steeringAngles.FL) + Fx_FR * std::sin(this->steeringAngles.FR))
    + 0.5 * this->sr * (Fx_RR * std::cos(this->steeringAngles.RR) - Fx_RL * std::cos(this->steeringAngles.RL))
    - this->lr * (Fx_RL * std::sin(this->steeringAngles.RL) + Fx_RR * std::sin(this->steeringAngles.RR));

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


    Eigen::Vector3d VehicleModelBicycle::getDynamicStates(double dt, Wheels frictionCoefficients)
    {
        double l = this->lr + this->lf;
        double vx = this->velocity.x();
        double vy = this->velocity.y();
        double ax = this->acceleration.x();
        double ay = this->acceleration.y();
        double r = this->angularVelocity.z();
        // Downforce
        double F_aero_downforce
            = 0. * 1.29 * this->aeroModel.aeroArea * this->aeroModel.cla * (vx * vx) + this->powerGroundSetpoint * this->powertrainModel.powerGroundForce;
        double F_aero_drag = 0.5 * 1.29 * this->aeroModel.aeroArea * this->aeroModel.cda * (vx * vx);
        double g = 9.81;
        double steeringFront = 0.5 * (this->steeringAngles.FL + this->steeringAngles.FR);

        // max because lifted tire makes no forces
        double Fz_Front = std::max(0.0, ((m * g + F_aero_downforce) * 0.5 * this->lr / l));
        double Fz_Rear = std::max(0.0, ((m * g + F_aero_downforce) * 0.5 * this->lf / l));

        Eigen::Vector3d vCog = this->velocity;
        Eigen::Vector3d omega = this->angularVelocity;

        Eigen::Vector3d rFL = Eigen::Vector3d(lf, 0.5 * sf, 0.0);
        Eigen::Vector3d rFR = Eigen::Vector3d(lf, -0.5 * sf, 0.0);
        Eigen::Vector3d rRL = Eigen::Vector3d(-lr, 0.5 * sr, 0.0);
        Eigen::Vector3d rRR = Eigen::Vector3d(-lr, -0.5 * sr, 0.0);
        Eigen::Vector3d rFront = Eigen::Vector3d(lf, 0.0, 0.0);
        Eigen::Vector3d rRear = Eigen::Vector3d(-lf, 0.0, 0.0);

        Eigen::Vector3d vFL = vCog + omega.cross(rFL);
        Eigen::Vector3d vFR = vCog + omega.cross(rFR);
        Eigen::Vector3d vRL = vCog + omega.cross(rRL);
        Eigen::Vector3d vRR = vCog + omega.cross(rRR);
        Eigen::Vector3d vFront = vCog + omega.cross(rFront);
        Eigen::Vector3d vRear = vCog + omega.cross(rRear);

        double rpm2ms = this->powertrainModel.wheelRadius * TWO_PI / 60.0;

        bool stillstand = (vCog.norm() < 0.1) && (std::abs(this->angularVelocity.z()) < 0.001);

        // tire side slip angles
        double eps = 0.00001;

        double kappaFront = std::atan2(vFront.y(), std::max(std::abs(vFront.x()), eps)) - steeringFront;
        double kappaRear = std::atan2(vRear.y(), std::max(std::abs(vRear.x()), eps));

        // don't steer when vehicle doesn't move
        if (stillstand)
        {
            kappaFront = 0.0;
            kappaRear = 0.0;
        }

        // old calculation as backup for now
        double Fx_FL = this->powertrainModel.gearRatio * this->torques.FL / this->powertrainModel.wheelRadius;
        double Fx_FR = this->powertrainModel.gearRatio * this->torques.FR / this->powertrainModel.wheelRadius;
        double Fx_RL = this->powertrainModel.gearRatio * this->torques.RL / this->powertrainModel.wheelRadius;
        double Fx_RR = this->powertrainModel.gearRatio * this->torques.RR / this->powertrainModel.wheelRadius;

        Fx_FL *= (((this->torques.FL) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        Fx_FR *= (((this->torques.FR) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        Fx_RL *= (((this->torques.RL) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        Fx_RR *= (((this->torques.RR) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;

        double Dlat_Front = 0.5 * (frictionCoefficients.FL + frictionCoefficients.FR) * this->tireModel.Dlat * Fz_Front;
        double Dlat_Rear = 0.5 * (frictionCoefficients.RL + frictionCoefficients.RR) * this->tireModel.Dlat * Fz_Rear;

        double Fy_Front = Dlat_Front * processSlipAngleLat(kappaFront);
        double Fy_Rear = Dlat_Rear * processSlipAngleLat(kappaRear);

        // todo convert speed
        this->wheelspeeds.FL = vFL.x() / rpm2ms;
        this->wheelspeeds.FR = vFR.x() / rpm2ms;
        this->wheelspeeds.RL = vRL.x() / rpm2ms;
        this->wheelspeeds.RR = vRR.x() / rpm2ms;

        double axTires = (std::cos(this->steeringAngles.FL) * Fx_FL + std::cos(this->steeringAngles.FR) * Fx_FR + Fx_RL
                             + Fx_RR - std::sin(steeringFront) * Fy_Front)
            / m;
        double axModel = axTires - F_aero_drag / m;

        double ayTires = (std::sin(this->steeringAngles.FL) * Fx_FL + std::sin(this->steeringAngles.FR) * Fx_FR
                             + std::cos(steeringFront) * Fy_Front + Fy_Rear)
            / m;
        double ayModel = (ayTires);

        double rdotFx
            = 0.5 * this->sf * (-Fx_FL * std::cos(this->steeringAngles.FL) + Fx_FR * std::cos(this->steeringAngles.FR))
            + this->lf * (Fx_FL * std::sin(this->steeringAngles.FL) + Fx_FR * std::sin(this->steeringAngles.FR))
            + 0.5 * this->sr * (Fx_RR * std::cos(this->steeringAngles.RR) - Fx_RL * std::cos(this->steeringAngles.RL))
            - this->lr * (Fx_RL * std::sin(this->steeringAngles.RL) + Fx_RR * std::sin(this->steeringAngles.RR));
        double rdotFy = this->lf * (Fy_Front * std::cos(steeringFront)) - this->lr * (Fy_Rear);
        double rdot = (1 / Izz * (rdotFx + rdotFy));

        Eigen::Vector3d ret(axModel, ayModel, rdot);
        return ret;
    }

void VehicleModelBicycle::forwardIntegrate(double dt, Wheels frictionCoefficients) {

  // Override dt with the difference of timestamps using the system clock
  //double dt1 = dt;
  
  static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = now - last_time;
  dt = elapsed.count();
  last_time = now;

  Eigen::Vector3d friction(std::min(200.0, 2000.0 * std::abs(this->velocity.x())),
    std::min(200.0, 2000.0 * std::abs(this->velocity.y())),
    std::min(200.0, 2000.0 * std::abs(this->velocity.z())));
  friction[0] = (this->velocity.x() > 0) ? friction.x() : -friction.x();
  friction[1] = (this->velocity.y() > 0) ? friction.y() : -friction.y();
  friction[2] = (this->velocity.z() > 0) ? friction.z() : -friction.z();
  // Update position based on velocity and orientation
  Eigen::AngleAxisd yawAngle(orientation.z(), Eigen::Vector3d::UnitZ());
  position += (yawAngle.matrix() * velocity) * dt;

  // Convert throttle to wheel torques (using PowertrainModel)
  powertrainModel.calculateWheelTorques(throttleActuation, torques);

  // Calculate dynamic states
  Eigen::Vector3d dynamicStates = getDynamicStates(dt, frictionCoefficients);

  // Update orientation based on angular velocity
  orientation.z() += dt * angularVelocity.z() * 10;

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