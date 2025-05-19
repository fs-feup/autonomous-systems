#pragma once

#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include "types.hpp"
#include "configParser.hpp"

// Interface for all vehicle models
class IVehicleModel {
public:
    virtual ~IVehicleModel() = default;

    // Configuration
    virtual bool readConfig(class ConfigElement& config) = 0;

    // State getters
    virtual Eigen::Vector3d getPosition() = 0;
    virtual Eigen::Vector3d getOrientation() = 0;
    virtual Eigen::Vector3d getVelocity() = 0;
    virtual Eigen::Vector3d getAcceleration() = 0;
    virtual Eigen::Vector3d getAngularVelocity() = 0;
    virtual Eigen::Vector3d getAngularAcceleration() = 0;
    
    virtual Wheels getSteeringAngles() = 0;
    virtual double getSteeringWheelAngle() = 0;
    virtual double getVoltageTS() = 0;
    virtual double getCurrentTS() = 0;
    virtual Wheels getWheelspeeds() = 0;
    virtual Wheels getWheelOrientations() = 0;
    virtual Wheels getTorques() = 0;
    
    // State setters
    virtual void setTorques(Wheels in) = 0;
    virtual void setRpmSetpoints(Wheels in) = 0;
    virtual void setMaxTorques(Wheels in) = 0;
    virtual void setMinTorques(Wheels in) = 0;
    virtual void setSteeringSetpointFront(double in) = 0;
    virtual void setSteeringSetpointRear(double in) = 0;
    virtual void setThrottle(double in) = 0;
    virtual void setPowerGroundSetpoint(double in) = 0;
    virtual void setPosition(Eigen::Vector3d position) = 0;
    virtual void setOrientation(Eigen::Vector3d orientation) = 0;
    
    // Simulation methods
    virtual void forwardIntegrate(double dt, Wheels frictionCoefficients) = 0;
    virtual std::array<Eigen::Vector3d, 4> getWheelPositions() = 0;
};