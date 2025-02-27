#include "VehicleModel/VehicleModelInterface.hpp"
#include "VehicleModel/VehicleModeConfig.hpp"
#include "VehicleModel/VehicleModelBicycle.hpp"
#include <cmath>

#include "transform.hpp"
VehicleModelBicycle::VehicleModelBicycle()
    : position(Eigen::Vector3d::Zero()),
      orientation(Eigen::Vector3d::Zero()),
      velocity(Eigen::Vector3d::Zero()),
      angularVelocity(Eigen::Vector3d::Zero()),
      acceleration(Eigen::Vector3d::Zero())
{
}

bool VehicleModelBicycle::readConfig(ConfigElement& config)
{
    auto configModel = config["simple_bicycle_model"];
    
    // Read kinematics parameters
    auto kinematics = configModel["kinematics"];
    kinematics.getElement<double>(&lf, "lf");
    kinematics.getElement<double>(&lr, "lr");
    kinematics.getElement<double>(&sf, "sf");
    kinematics.getElement<double>(&sr, "sr");

    // Read tire model parameters
    auto tire = configModel["tire"];
    tire.getElement<double>(&Blat, "Blat");
    tire.getElement<double>(&Clat, "Clat");
    tire.getElement<double>(&Dlat, "Dlat");
    tire.getElement<double>(&Elat, "Elat");

    // Read aerodynamic parameters
    auto aero = configModel["aero"];
    aero.getElement<double>(&cla, "cla");
    aero.getElement<double>(&cda, "cda");
    aero.getElement<double>(&aeroArea, "aeroArea");

    // Read general vehicle parameters
    configModel.getElement<double>(&m, "m");
    configModel.getElement<double>(&Izz, "Izz");
    configModel.getElement<double>(&wheelRadius, "wheelRadius");
    configModel.getElement<double>(&gearRatio, "gearRatio");
    configModel.getElement<double>(&innerSteeringRatio, "innerSteeringRatio");
    configModel.getElement<double>(&outerSteeringRatio, "outerSteeringRatio");
    configModel.getElement<double>(&nominalVoltageTS, "nominalVoltageTS");
    configModel.getElement<double>(&powerGroundForce, "powerGroundForce");
    
    return true;
}

Eigen::Vector3d VehicleModelBicycle::getPosition()
{
    return position;
}

Eigen::Vector3d VehicleModelBicycle::getOrientation()
{
    return orientation;
}

Eigen::Vector3d VehicleModelBicycle::getVelocity()
{
    return velocity;
}

Eigen::Vector3d VehicleModelBicycle::getAcceleration()
{
    return acceleration;
}

Eigen::Vector3d VehicleModelBicycle::getAngularVelocity()
{
    return angularVelocity;
}

Eigen::Vector3d VehicleModelBicycle::getAngularAcceleration()
{
    return angularAcceleration;
}

Wheels VehicleModelBicycle::getSteeringAngles()
{
    return steeringAngles;
}

double VehicleModelBicycle::getSteeringWheelAngle()
{
    // Calculate steering wheel angle based on the front left steering angle and appropriate ratio
    return (steeringAngles.FL > 0) ? 
           steeringAngles.FL / innerSteeringRatio : 
           steeringAngles.FL / outerSteeringRatio;
}

double VehicleModelBicycle::getVoltageTS()
{
    return nominalVoltageTS;
}

double VehicleModelBicycle::getCurrentTS()
{
    constexpr double powerCoeff = 1.0 / 9.55;
    
    // Calculate power for each wheel
    double powerFL = torques.FL * wheelspeeds.FL * powerCoeff;
    double powerFR = torques.FR * wheelspeeds.FR * powerCoeff;
    double powerRL = torques.RL * wheelspeeds.RL * powerCoeff;
    double powerRR = torques.RR * wheelspeeds.RR * powerCoeff;
    
    // Calculate powertrain efficiency based on rear wheel torques
    double powertrainEfficiency = 1.0;
    
    // Calculate total power and return current
    double totalPower = (powerFL + powerFR + powerRL + powerRR) / powertrainEfficiency;
    return (totalPower / nominalVoltageTS);
}

Wheels VehicleModelBicycle::getWheelspeeds()
{
    return wheelspeeds;
}

Wheels VehicleModelBicycle::getWheelOrientations()
{
    return wheelOrientations;
}

Wheels VehicleModelBicycle::getTorques()
{
    return torques;
}

void VehicleModelBicycle::setTorques(Wheels in)
{
    torques = in;
}

void VehicleModelBicycle::setRpmSetpoints(Wheels in)
{
    rpmSetpoints = in;
}

void VehicleModelBicycle::setMaxTorques(Wheels in)
{
    maxTorques = in;
}

void VehicleModelBicycle::setMinTorques(Wheels in)
{
    minTorques = in;
}

void VehicleModelBicycle::setSteeringSetpointFront(double in)
{
    setSteeringFront(in);
}

void VehicleModelBicycle::setSteeringSetpointRear(double in)
{
    // Rear steering not implemented
}

void VehicleModelBicycle::setThrottle(double in)
{
    throttleActuation = std::clamp(in, -1.0, 1.0);
}

void VehicleModelBicycle::setPowerGroundSetpoint(double in)
{
    powerGroundSetpoint = std::clamp(in, 0.0, 1.0);
}

void VehicleModelBicycle::setSteeringFront(double in)
{
    // Apply Ackermann steering geometry
    double avgRatio = 0.5 * (innerSteeringRatio + outerSteeringRatio);
    
    if (in > 0) {
        steeringAngles.FL = innerSteeringRatio * in / avgRatio;
        steeringAngles.FR = outerSteeringRatio * in / avgRatio;
    } else {
        steeringAngles.FL = outerSteeringRatio * in / avgRatio;
        steeringAngles.FR = innerSteeringRatio * in / avgRatio;
    }
}

void VehicleModelBicycle::setPosition(Eigen::Vector3d newPosition)
{
    position = newPosition;
}

void VehicleModelBicycle::setOrientation(Eigen::Vector3d newOrientation)
{
    orientation = newOrientation;
}

int VehicleModelBicycle::sign(double value)
{
    if (value > 0)
        return 1;
    if (value < 0)
        return -1;
    return 1;  // Default to 1 for zero
}

double VehicleModelBicycle::processSlipAngleLat(double alpha_input, double Fz)
{
    using namespace VehicleModelConstants;

    double dpi = (p_input - NOMPRES) / NOMPRES;
    double Fz_0_prime = LFZO * FNOM;
    double Kya = PKY1 * Fz_0_prime * (1 + PPY1 * dpi) * (1 - PKY3 * abs(y_input))
        * sin(PKY4 * atan((Fz / Fz_0_prime) / ((PKY2 + PKY5 * y_input * y_input) * (1 + PPY2 * dpi)))) * zeta_3
        * LKY;
    double dfz = (Fz - Fz_0_prime) / Fz_0_prime;
    double Kyg0 = Fz * (PKY6 + PKY7 * dfz) * (1 + PPY5 * dpi) * LKYC;
    double Vs_y = tan(alpha_input) * abs(V_cx);
    double Vs_x = -slip_ratio * abs(V_cx);
    double Vs = sqrt(Vs_x * Vs_x + Vs_y * Vs_y);
    double V0 = LONGVL;
    double LMUY_star = LMUY / (1.0 + LMUV * (Vs / V0));
    double SVyg = Fz * (PVY3 + PVY4 * dfz) * y_input * LKYC * LMUY_star * zeta_2;
    double SHy
        = (PHY1 + PHY2 * dfz) + ((Kyg0 * y_input - SVyg) / (Kya + eps_k * sign(Kya))) * zeta_0 + zeta_4 - 1.0;
    double alpha_y = alpha_input + SHy;
    double Ey
        = (PEY1 + PEY2 * dfz) * (1 + PEY5 * y_input * y_input - (PEY3 + PEY4 * y_input) * sign(alpha_y)) * LEY;
    double Cy = PCY1 * LCY;
    double mu_y = (PDY1 + PDY2 * dfz) * (1.0 + PPY3 * dpi + PPY4 * dpi * dpi) * (1.0 - PDY3 * y_input * y_input)
        * LMUY_star;
    double Dy = mu_y * Fz * zeta_2;
    double By = Kya / (Cy * Dy + eps_y * sign(Dy));
    double DVyk = mu_y * Fz * (RVY1 + RVY2 * dfz + RVY3 * y_input) * cos(atan(RVY4 * alpha_input)) * zeta_2;
    double SVyk = DVyk * sin(RVY5 * atan(RVY6 * slip_ratio)) * LVYKA;
    double Eyk = REY1 + REY2 * dfz;
    double SHyk = RHY1 + RHY2 * dfz;
    double ks = slip_ratio + SHyk;
    double Byk = (RBY1 + RBY4 * y_input * y_input) * cos(atan(RBY2 * (alpha_input - RBY3))) * LYKA;
    double Cyk = RCY1;
    double Gyk_0 = cos(Cyk * atan(Byk * SHyk - Eyk * (Byk * SHyk - atan(Byk * SHyk))));
    double Gyk = (cos(Cyk * atan(Byk * ks - Eyk * (Byk * ks - atan(Byk * ks)))) / Gyk_0);
    double SVy = Fz * (PVY1 + PVY2 * dfz) * LVY * LMUY_star * zeta_2 + SVyg;
    double Fy_0 = Dy * sin(Cy * atan(By * alpha_y - Ey * (By * alpha_y - atan(By * alpha_y)))) + SVy;
    double Fy = Gyk * Fy_0 + SVyk;

    return Fy;
}

Eigen::Vector3d VehicleModelBicycle::getDynamicStates(double dt)
{
    constexpr double g = 9.81;
    constexpr double airDensity = 1.29;

    // Calculate average front steering angle
    double steeringFront = 0.5 * (steeringAngles.FL + steeringAngles.FR);
    
    // Calculate total vehicle length
    double l = lr + lf;
    
    // Calculate aerodynamic forces
    double velocitySquared = velocity.x() * velocity.x();
    double F_aero_downforce = 0.5 * airDensity * aeroArea * cla * velocitySquared + 
                               powerGroundSetpoint * powerGroundForce;
    double F_aero_drag = 0.5 * airDensity * aeroArea * cda * velocitySquared;
    
    // Calculate normal forces on axles
    double Fz_Front = std::max(0.0, (m * g + F_aero_downforce) * 0.5 * lr / l);
    double Fz_Rear = std::max(0.0, (m * g + F_aero_downforce) * 0.5 * lf / l);

    // Calculate rolling resistance (applied opposite to velocity direction)
    double frict = (Fz_Front + Fz_Rear) * 0.015;
    Eigen::Vector3d friction = (velocity.norm() < 0.01) ?
                               Eigen::Vector3d::Zero() :
                               Eigen::Vector3d(frict, 0.0, 0.0);

    // Calculate wheel positions relative to CoG
    Eigen::Vector3d rFL(lf, 0.5 * sf, 0.0);
    Eigen::Vector3d rFR(lf, -0.5 * sf, 0.0);
    Eigen::Vector3d rRL(-lr, 0.5 * sr, 0.0);
    Eigen::Vector3d rRR(-lr, -0.5 * sr, 0.0);
    Eigen::Vector3d rFront(lf, 0.0, 0.0);
    Eigen::Vector3d rRear(-lr, 0.0, 0.0);

    // Calculate wheel velocities
    Eigen::Vector3d vFL = velocity + angularVelocity.cross(rFL);
    Eigen::Vector3d vFR = velocity + angularVelocity.cross(rFR);
    Eigen::Vector3d vRL = velocity + angularVelocity.cross(rRL);
    Eigen::Vector3d vRR = velocity + angularVelocity.cross(rRR);
    Eigen::Vector3d vFront = velocity + angularVelocity.cross(rFront);
    Eigen::Vector3d vRear = velocity + angularVelocity.cross(rRear);

    // Conversion factor from RPM to m/s
    double rpm2ms = wheelRadius * 2.0 * M_PI / 60;

    // Check if vehicle is at standstill
    bool stillstand = (velocity.norm() < 0.1) && (std::abs(angularVelocity.z()) < 0.001);

    // Calculate powertrain efficiency
    double powertrainEfficiency = 0.002333 * (torques.RL + torques.RR) + 0.594;
    
    // Calculate longitudinal forces on wheels
    double Fx_FL = 0; // Front wheel drive not enabled
    double Fx_FR = 0; // Front wheel drive not enabled
    double Fx_RL = (gearRatio * torques.RL / wheelRadius) * powertrainEfficiency;
    double Fx_RR = (gearRatio * torques.RR / wheelRadius) * powertrainEfficiency;
    
    // Apply forces only when torque is significant or vehicle is moving
    Fx_FL *= (torques.FL > 0.5 || velocity.x() > 0.3) ? 1.0 : 0.0;
    Fx_FR *= (torques.FR > 0.5 || velocity.x() > 0.3) ? 1.0 : 0.0;
    Fx_RL *= (torques.RL > 0.5 || velocity.x() > 0.3) ? 1.0 : 0.0;
    Fx_RR *= (torques.RR > 0.5 || velocity.x() > 0.3) ? 1.0 : 0.0;
    
    // Calculate tire slip angles
    constexpr double eps = 0.00001; // Prevent division by zero
    
    // Calculate slip angles for front and rear
    double kappaFront = stillstand ? 0.0 : 
                        std::atan2(vFront.y(), std::max(std::abs(vFront.x()), eps)) - steeringFront;
    double kappaRear = stillstand ? 0.0 : 
                       std::atan2(vRear.y(), std::max(std::abs(vRear.x()), eps));
    
    // Calculate lateral forces using tire model
    double Fy_Front = processSlipAngleLat(kappaFront, Fz_Front/2) * 2;
    double Fy_Rear = processSlipAngleLat(kappaRear, Fz_Rear/2) * 2;
    
    // Update wheel speeds
    wheelspeeds.FL = vFL.x() / rpm2ms;
    wheelspeeds.FR = vFR.x() / rpm2ms;
    wheelspeeds.RL = vRL.x() / rpm2ms;
    wheelspeeds.RR = vRR.x() / rpm2ms;
    
    // Calculate longitudinal acceleration from tire forces
    double axTires = (std::cos(steeringAngles.FL) * Fx_FL + 
                      std::cos(steeringAngles.FR) * Fx_FR + 
                      Fx_RL + Fx_RR - 
                      std::sin(steeringFront) * Fy_Front) / m;
                      
    // Apply aerodynamic drag and friction to get total acceleration
    double axModel = axTires - F_aero_drag / m - friction.x() / m;
    
    // Calculate lateral acceleration from tire forces
    double ayTires = (std::sin(steeringAngles.FL) * Fx_FL + 
                      std::sin(steeringAngles.FR) * Fx_FR +
                      std::cos(steeringFront) * Fy_Front + 
                      Fy_Rear) / m;
    double ayModel = ayTires;
    
    // Calculate yaw moment contributions
    double rdotFx = 0.5 * sf * (-Fx_FL * std::cos(steeringAngles.FL) + 
                                Fx_FR * std::cos(steeringAngles.FR)) +
                    lf * (Fx_FL * std::sin(steeringAngles.FL) + 
                          Fx_FR * std::sin(steeringAngles.FR)) +
                    0.5 * sr * (Fx_RR - Fx_RL);
                    
    double rdotFy = lf * (Fy_Front * std::cos(steeringFront)) - 
                    lr * (Fy_Rear);
                    
    // Calculate yaw acceleration
    double rdot = (rdotFx + rdotFy) / Izz;
    
    return Eigen::Vector3d(axModel, ayModel, rdot);
}

void VehicleModelBicycle::forwardIntegrate(double dt)
{
    // Update position based on velocity and orientation
    Eigen::AngleAxisd yawAngle(orientation.z(), Eigen::Vector3d::UnitZ());
    position += (yawAngle.matrix() * velocity) * dt;
    
    // Convert throttle to wheel torques
    // Rear wheel drive with even torque distribution
    const double maxTorque = 120.0; // 120 Nm max total torque
    torques.FL = 0;
    torques.FR = 0;
    torques.RL = throttleActuation * maxTorque * 0.5;
    torques.RR = throttleActuation * maxTorque * 0.5;
    
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
    const double wheelRotationFactor = 2.0 * M_PI / (60.0 * gearRatio);
    
    wheelOrientations.FL = std::fmod(
        wheelOrientations.FL + wheelspeeds.FL * dt * wheelRotationFactor, 2.0 * M_PI);
    wheelOrientations.FR = std::fmod(
        wheelOrientations.FR + wheelspeeds.FR * dt * wheelRotationFactor, 2.0 * M_PI);
    wheelOrientations.RL = std::fmod(
        wheelOrientations.RL + wheelspeeds.RL * dt * wheelRotationFactor, 2.0 * M_PI);
    wheelOrientations.RR = std::fmod(
        wheelOrientations.RR + wheelspeeds.RR * dt * wheelRotationFactor, 2.0 * M_PI);
}

std::array<Eigen::Vector3d, 4> VehicleModelBicycle::getWheelPositions()
{
    // Transform wheel positions from vehicle coordinates to world coordinates
    auto rotMat = eulerAnglesToRotMat(orientation).transpose();
    
    Eigen::Vector3d FL = rotMat * Eigen::Vector3d(lf, sf * 0.5, 0.0) + position;
    Eigen::Vector3d FR = rotMat * Eigen::Vector3d(lf, -sf * 0.5, 0.0) + position;
    Eigen::Vector3d RL = rotMat * Eigen::Vector3d(-lr, sr * 0.5, 0.0) + position;
    Eigen::Vector3d RR = rotMat * Eigen::Vector3d(-lr, -sr * 0.5, 0.0) + position;
    
    return {FL, FR, RL, RR};
}
