#pragma once

#include "VehicleModel/VehicleModelInterface.hpp"
#include <Eigen/Dense>
#include <cmath>

class VehicleModelBicycle : public IVehicleModel {
public:

    // Physical constants
    static constexpr double GRAVITY = 9.81;
    static constexpr double ROLLING_RESISTANCE_COEFF = 0.015;
    static constexpr double AIR_DENSITY = 1.29;
    static constexpr double MAX_TORQUE = 120.0; // Nm
    
    // Threshold constants
    static constexpr double VELOCITY_THRESHOLD = 0.1;
    static constexpr double ANGULAR_VELOCITY_THRESHOLD = 0.001;
    static constexpr double TORQUE_THRESHOLD = 0.5;
    static constexpr double VELOCITY_MIN_THRESHOLD = 0.3;

    // Mathematical constants
    static constexpr double TWO_PI = 2.0 * M_PI;

    VehicleModelBicycle();
    
    // Interface implementation
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
    // Component-based design
    struct AerodynamicsModel {
        double cla;       // Lift coefficient
        double cda;       // Drag coefficient
        double aeroArea;  // Reference area

        // Calculate aerodynamic forces
        void calculateForces(double velocityX, double& downforce, double& drag) const;
    };

    struct TireModel {
        // Basic Magic Formula parameters
        double Blat, Clat, Dlat, Elat;
        
        // Magic Formula parameters
        double NOMPRES = 83000;
        double FNOM = 1110;
        double LFZO = 1.0;
    
        // Slip calculation constants
        double eps_k = 1e-6;
        double eps_y = 1e-6;
        double LONGVL = 10.0; 
        double LMUY = 1.0;
        double LVY = 1.0;
        double LMUV = 0.0;
        
        // Lateral force coefficients
        double LKYC = 1.0;
        double LCY = 1.0;
        double LEY = 1.0;
        double LYKA = 1.0;
        double LVYKA = 1.0;
        double LKY = 1.0;
        
        // P-coefficients
        double PPY1 = 0.43899;
        double PPY2 = 1.3335;
        double PPY3 = -0.15166;
        double PPY4 = 0.053855;
        double PPY5 = -0.81712;
        
        double PVY1 = 0.047901;
        double PVY2 = 0.014419;
        double PVY3 = -0.3046;
        double PVY4 = 1.4794;
        
        double PKY1 = -39.1199;
        double PKY2 = 1.6728;
        double PKY3 = 0.86703;
        double PKY4 = 2.0;
        double PKY5 = 29.7896;
        double PKY6 = -4.7914;
        double PKY7 = -2.0621;
        
        double PDY1 = 2.3352;
        double PDY2 = -0.37521;
        double PDY3 = 10.0;
        
        double PHY1 = 0.00013682;
        double PHY2 = -0.00046485;
        
        double PEY1 = 0.68111;
        double PEY2 = -0.35401;
        double PEY3 = -0.080852;
        double PEY4 = -5.562;
        double PEY5 = -64.1838;
        
        double PCY1 = 1.8528;
        
        // R-coefficients
        double RCY1 = 1.0;
        
        double RBY1 = 5.0;
        double RBY2 = 2.0;
        double RBY3 = 0.02;
        double RBY4 = 0.0;
        
        double REY1 = -0.1;
        double REY2 = 0.1;
        
        double RHY1 = 0.0;
        double RHY2 = 0.0;
        
        double RVY1 = 0.0;
        double RVY2 = 0.0;
        double RVY3 = 0.0;
        double RVY4 = 0.0;
        double RVY5 = 0.0;
        double RVY6 = 0.0;
        
        // Variables for calculation
        double p_input = 82737;
        double y_input = 0.0;
        double slip_ratio = 0.0;
        double V_cx = 1.0;
        
        // Zeta factors
        double zeta_0 = 1.0;
        double zeta_2 = 1.0;
        double zeta_3 = 1.0;
        double zeta_4 = 1.0;
        
        // Calculate lateral force using Magic Formula
        double calculateLateralForce(double slipAngle, double normalForce) const;
        int sign(double value) const;
    };

    struct PowertrainModel {
        double gearRatio;
        double wheelRadius;
        double nominalVoltageTS;
        double powerGroundForce;
        
        // Calculate torques from throttle
        void calculateWheelTorques(double throttleInput, Wheels& torques) const;
        
        // Calculate powertrain efficiency
        double calculateEfficiency(const Wheels& torques) const;
        
        // Calculate current
        double calculateCurrent(const Wheels& torques, const Wheels& wheelspeeds, double voltage) const;
    };

    struct SteeringModel {
        double innerSteeringRatio;
        double outerSteeringRatio;
        
        // Apply Ackermann steering geometry
        void calculateSteeringAngles(double steeringInput, Wheels& steeringAngles) const;
        
        // Calculate steering wheel angle
        double calculateSteeringWheelAngle(const Wheels& steeringAngles) const;
    };

    void calculateWheelGeometry(
        double& steeringFront,
        Eigen::Vector3d& rFL, Eigen::Vector3d& rFR,
        Eigen::Vector3d& rRL, Eigen::Vector3d& rRR,
        Eigen::Vector3d& rFront, Eigen::Vector3d& rRear,
        Eigen::Vector3d& vFL, Eigen::Vector3d& vFR,
        Eigen::Vector3d& vRL, Eigen::Vector3d& vRR) const;
        
    void calculateLongitudinalForces(
        double& Fx_FL, double& Fx_FR, double& Fx_RL, double& Fx_RR) const;
        
    Eigen::Vector3d calculateAccelerations(
        double steeringFront,
        double Fx_FL, double Fx_FR, double Fx_RL, double Fx_RR,
        double Fy_Front, double Fy_Rear,
        double drag, const Eigen::Vector3d& friction) const;
        
    void updateWheelSpeeds(
        const Eigen::Vector3d& vFL, const Eigen::Vector3d& vFR,
        const Eigen::Vector3d& vRL, const Eigen::Vector3d& vRR);

    // Physics calculation methods
    Eigen::Vector3d getDynamicStates(double dt);
    void calculateNormalForces(double& Fz_Front, double& Fz_Rear) const;
    void calculateWeightTransfer(double& Fz_Front, double& Fz_Rear , double& Fx_FL, double& Fx_FR, double& Fx_RL, double& Fx_RR) const;
    void calculateSlipAngles(double& kappaFront, double& kappaRear) const;
    void updateWheelSpeeds(double dt);
    double processSlipAngleLat(double alpha_input, double Fz);

    // Vehicle state variables
    Eigen::Vector3d position = Eigen::Vector3d::Zero();         // world x, y, z
    Eigen::Vector3d orientation  = Eigen::Vector3d::Zero();      // roll, pitch, yaw
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();         // body x, y, z
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();  // roll rate, pitch rate, yaw rate
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();     // body x, y, z
    Eigen::Vector3d angularAcceleration = Eigen::Vector3d::Zero(); // roll acc, pitch acc, yaw acc
    
    // Wheel state
    Wheels steeringAngles = {0.0, 0.0, 0.0, 0.0, 0.0}; // FL, FR, RL, RR
    Wheels wheelspeeds = {0.0, 0.0, 0.0, 0.0, 0.0};
    Wheels wheelOrientations = {0.0, 0.0, 0.0, 0.0, 0.0};
    Wheels torques = {0.0, 0.0, 0.0, 0.0, 0.0};
    Wheels rpmSetpoints = {0.0, 0.0, 0.0, 0.0, 0.0};
    Wheels maxTorques = {0.0, 0.0, 0.0, 0.0, 0.0};
    Wheels minTorques = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    // Input state
    double throttleActuation = 0.0;
    double powerGroundSetpoint = 0.0;
    
    // Vehicle parameters
    double lf, lr;  // Distance from CoG to front/rear axle
    double sf, sr;  // Track width front/rear
    double m;       // Vehicle mass
    double Izz;     // Yaw moment of inertia
    
    // Component models
    AerodynamicsModel aeroModel;
    TireModel tireModel;
    PowertrainModel powertrainModel;
    SteeringModel steeringModel;
};