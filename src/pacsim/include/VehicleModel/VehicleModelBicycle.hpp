#pragma once

#include "VehicleModel/VehicleModelInterface.hpp"
#include "VehicleModel/VehicleModeConfig.hpp"
#include "transform.hpp"

#include <cmath>

/**
 * Bicycle model implementation of the vehicle model interface.
 * This model simulates vehicle dynamics using a simplified bicycle model
 * with tire physics for lateral forces.
 */
class VehicleModelBicycle : public IVehicleModel {
public:
    VehicleModelBicycle();
    ~VehicleModelBicycle() override = default;

    // Configuration
    bool readConfig(ConfigElement& config) override;

    // State getters
    Eigen::Vector3d getPosition() override;
    Eigen::Vector3d getOrientation() override;
    Eigen::Vector3d getVelocity() override;
    Eigen::Vector3d getAcceleration() override;
    Eigen::Vector3d getAngularVelocity() override;
    Eigen::Vector3d getAngularAcceleration() override;
    
    Wheels getSteeringAngles() override;
    double getSteeringWheelAngle() override;
    double getVoltageTS() override;
    double getCurrentTS() override;
    Wheels getWheelspeeds() override;
    Wheels getWheelOrientations() override;
    Wheels getTorques() override;
    
    // State setters
    void setTorques(Wheels in) override;
    void setRpmSetpoints(Wheels in) override;
    void setMaxTorques(Wheels in) override;
    void setMinTorques(Wheels in) override;
    void setSteeringSetpointFront(double in) override;
    void setSteeringSetpointRear(double in) override;
    void setThrottle(double in) override;
    void setPowerGroundSetpoint(double in) override;
    void setPosition(Eigen::Vector3d position) override;
    void setOrientation(Eigen::Vector3d orientation) override;
    
    // Simulation methods
    void forwardIntegrate(double dt) override;
    std::array<Eigen::Vector3d, 4> getWheelPositions() override;

private:
    // Helper methods
    Eigen::Vector3d getDynamicStates(double dt);
    void setSteeringFront(double in);
    int sign(double value);
    double processSlipAngleLat(double alpha_input, double Fz);

private:
    // Vehicle state
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d orientation = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularAcceleration = Eigen::Vector3d::Zero();
    
    Wheels torques{};
    Wheels steeringAngles{};
    Wheels wheelOrientations{};
    Wheels wheelspeeds{};

    // Control inputs
    double throttleActuation = 0.0;
    double powerGroundSetpoint = 0.0;
    
    // Configuration parameters
    // Chassis dimensions
    double lr = 0.0;  // Distance from CoG to rear axle
    double lf = 0.0;  // Distance from CoG to front axle
    double sf = 0.0;  // Track width front
    double sr = 0.0;  // Track width rear

    // Tire model parameters
    double Blat = 0.0;
    double Clat = 0.0;
    double Dlat = 0.0;
    double Elat = 0.0;

    // Aerodynamic parameters
    double cla = 0.0;
    double cda = 0.0;
    double aeroArea = 0.0;

    // Vehicle mass and inertia
    double m = 0.0;
    double Izz = 0.0;

    // Wheel and steering parameters
    double wheelRadius = 0.0;
    double gearRatio = 0.0;
    double innerSteeringRatio = 0.0;
    double outerSteeringRatio = 0.0;
    double nominalVoltageTS = 0.0;
    double powerGroundForce = 0.0;

    // Limits and setpoints
    Wheels minTorques{};
    Wheels maxTorques{};
    Wheels rpmSetpoints{};
    Wheels currentFx{};
    Wheels currentFy{};
};
