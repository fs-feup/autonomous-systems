#include "VehicleModel/VehicleModelInterface.hpp"
#include "VehicleModel/VehicleModeConfig.hpp"

#include "transform.hpp"
class VehicleModelBicycle : public IVehicleModel
{
public:
    VehicleModelBicycle()
    {
        // Initialize position, orientation, velocity, angular velocity, and acceleration to zero
        this->position = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->orientation = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->angularVelocity = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);

        // Initialize torques, steering angles, wheel orientations, and wheel speeds to zero
        this->torques = { 0.0, 0.0, 0.0, 0.0 };
        this->steeringAngles = { 0.0, 0.0, 0.0, 0.0 };
        this->wheelOrientations = { 0.0, 0.0, 0.0, 0.0 };
        this->wheelspeeds = { 0.0, 0.0, 0.0, 0.0 };
    }

    bool readConfig(ConfigElement& config)
    {
        // Read configuration parameters from the provided config element
        auto configModel = config["simple_bicycle_model"];
        configModel["kinematics"].getElement<double>(&this->lf, "lf");
        configModel["kinematics"].getElement<double>(&this->lr, "lr");
        configModel["kinematics"].getElement<double>(&this->sf, "sf");
        configModel["kinematics"].getElement<double>(&this->sr, "sr");

        configModel["tire"].getElement<double>(&this->Blat, "Blat");
        configModel["tire"].getElement<double>(&this->Clat, "Clat");
        configModel["tire"].getElement<double>(&this->Dlat, "Dlat");
        configModel["tire"].getElement<double>(&this->Elat, "Elat");

        configModel["aero"].getElement<double>(&this->cla, "cla");
        configModel["aero"].getElement<double>(&this->cda, "cda");
        configModel["aero"].getElement<double>(&this->aeroArea, "aeroArea");

        configModel.getElement<double>(&this->m, "m");
        configModel.getElement<double>(&this->Izz, "Izz");
        configModel.getElement<double>(&this->wheelRadius, "wheelRadius");
        configModel.getElement<double>(&this->gearRatio, "gearRatio");
        configModel.getElement<double>(&this->innerSteeringRatio, "innerSteeringRatio");
        configModel.getElement<double>(&this->outerSteeringRatio, "outerSteeringRatio");
        configModel.getElement<double>(&this->nominalVoltageTS, "nominalVoltageTS");
        configModel.getElement<double>(&this->powerGroundForce, "powerGroundForce");
        // configModel.getElement<double>(&this->powertrainEfficiency, "powertrainEfficiency");
        return true;
    }

    // Getters for various vehicle states
    Eigen::Vector3d getPosition()
    {
        Eigen::Vector3d ret = this->position;
        return ret;
    }

    Eigen::Vector3d getOrientation() { return this->orientation; }

    Eigen::Vector3d getVelocity() { return this->velocity; }

    Eigen::Vector3d getAcceleration() { return this->acceleration; }

    Eigen::Vector3d getAngularVelocity() { return this->angularVelocity; }

    Eigen::Vector3d getAngularAcceleration() { return this->angularAcceleration; }

    Wheels getSteeringAngles() { return this->steeringAngles; }

    double getSteeringWheelAngle()
    {
        // Calculate the steering wheel angle based on the front left steering angle and ratios (ackermann probably)
        return (this->steeringAngles.FL > 0) ? this->steeringAngles.FL / this->innerSteeringRatio
                                             : this->steeringAngles.FL / this->outerSteeringRatio;
    }

    double getVoltageTS() { return this->nominalVoltageTS; }

    double getCurrentTS()
    {
        double powerCoeff = 1.0 / 9.55;
        double powerFL = this->torques.FL * this->wheelspeeds.FL * powerCoeff;
        double powerFR = this->torques.FR * this->wheelspeeds.FR * powerCoeff;
        double powerRL = this->torques.RL * this->wheelspeeds.RL * powerCoeff;
        double powerRR = this->torques.RR * this->wheelspeeds.RR * powerCoeff;
        double powertrainEfficiency = 1.0;
        double totalPower = (powerFL + powerFR + powerRL + powerRR) / powertrainEfficiency;

        return (totalPower / this->nominalVoltageTS);
    }

    Wheels getWheelspeeds() { return this->wheelspeeds; }

    Wheels getWheelOrientations() { return this->wheelOrientations; }

    Wheels getTorques() { return this->torques; }

    // Setters for various vehicle states
    void setTorques(Wheels in) { this->torques = in; }

    void setRpmSetpoints(Wheels in) { this->rpmSetpoints = in; }

    void setMaxTorques(Wheels in) { this->maxTorques = in; }

    void setMinTorques(Wheels in) { this->minTorques = in; }

    void setSteeringSetpointFront(double in) { setSteeringFront(in); }

    void setSteeringSetpointRear(double in) { return; }

    void setThrottle(double in) { this->throttleActuation = std::clamp(in, -1.0, 1.0); }

    void setPowerGroundSetpoint(double in) { this->powerGroundSetpoint = std::min(std::max(in, 0.0), 1.0); }

    void setSteeringFront(double in)
    {
        // Set the front steering angles based on the input and steering ratios
        double avgRatio = 0.5 * (this->innerSteeringRatio + this->outerSteeringRatio);
        if (in > 0)
        {
            this->steeringAngles.FL = this->innerSteeringRatio * in / avgRatio;
            this->steeringAngles.FR = this->outerSteeringRatio * in / avgRatio;
        }
        else
        {
            this->steeringAngles.FL = this->outerSteeringRatio * in / avgRatio;
            this->steeringAngles.FR = this->innerSteeringRatio * in / avgRatio;
        }
        return;
    }

    void setPosition(Eigen::Vector3d position) { this->position = position; }
    void setOrientation(Eigen::Vector3d orientation) { this->orientation = orientation; }

    int sign(double value)
    {
        if (value > 0)
            return 1;
        if (value < 0)
            return -1;
        return 1; // For zero, return 1
    }

    double processSlipAngleLat(double alpha_input, double Fz)
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

    // Calculate dynamic states (ax, ay, rdot) based on the current state and time step
    Eigen::Vector3d getDynamicStates(double dt)
    {
        // Downforce
        double F_aero_downforce
        = 0.5 * 1.29 * this->aeroArea * this->cla * (this->velocity.x() * this->velocity.x()) + this->powerGroundSetpoint * this->powerGroundForce;
        double F_aero_drag = 0.5 * 1.29 * this->aeroArea * this->cda * (this->velocity.x() * this->velocity.x());
        double g = 9.81;
        double steeringFront = 0.5 * (this->steeringAngles.FL + this->steeringAngles.FR);
        
        // Calculate normal forces on the front and rear axles
        double l = this->lr + this->lf;
        double Fz_Front = std::max(0.0, ((m * g + F_aero_downforce) * 0.5 * this->lr / l));
        double Fz_Rear = std::max(0.0, ((m * g + F_aero_downforce) * 0.5 * this->lf / l));

        // Friction is zero at low speed so car doesnt jump between positive and negative velocities (oscillations)
        // If not then check direction of velocity and apply friction in opposite direction
        double frict = (Fz_Front + Fz_Rear) * 0.015;
        Eigen::Vector3d friction = (this->velocity.norm() < 0.01)
            ? Eigen::Vector3d::Zero()
            : Eigen::Vector3d(frict, 0.0, 0.0);

        Eigen::Vector3d vCog = this->velocity;
        Eigen::Vector3d omega = this->angularVelocity;

        // Position vectors of the wheels relative to the center of gravity
        Eigen::Vector3d rFL = Eigen::Vector3d(lf, 0.5 * sf, 0.0);
        Eigen::Vector3d rFR = Eigen::Vector3d(lf, -0.5 * sf, 0.0);
        Eigen::Vector3d rRL = Eigen::Vector3d(-lr, 0.5 * sr, 0.0);
        Eigen::Vector3d rRR = Eigen::Vector3d(-lr, -0.5 * sr, 0.0);
        Eigen::Vector3d rFront = Eigen::Vector3d(lf, 0.0, 0.0);
        Eigen::Vector3d rRear = Eigen::Vector3d(-lf, 0.0, 0.0);

        // Calculate the velocities of the wheels
        Eigen::Vector3d vFL = vCog + omega.cross(rFL);
        Eigen::Vector3d vFR = vCog + omega.cross(rFR);
        Eigen::Vector3d vRL = vCog + omega.cross(rRL);
        Eigen::Vector3d vRR = vCog + omega.cross(rRR);
        Eigen::Vector3d vFront = vCog + omega.cross(rFront);
        Eigen::Vector3d vRear = vCog + omega.cross(rRear);

        double rpm2ms = this->wheelRadius * 2.0 * M_PI / 60;

        // Check if the vehicle is at a standstill
        bool stillstand = (vCog.norm() < 0.1) && (std::abs(this->angularVelocity.z()) < 0.001);

        
        double powertrainEfficiency = 0.002333 * (this->torques.RL + this->torques.RR) + 0.594;
        // Calculate longitudinal forces on the wheels
        double Fx_FL = 0;
        double Fx_FR = 0;
        double Fx_RL = (this->gearRatio * this->torques.RL / (this->wheelRadius)) * powertrainEfficiency;
        double Fx_RR = (this->gearRatio * this->torques.RR / (this->wheelRadius)) * powertrainEfficiency;
        
        // Apply forces only if the torques are significant or the vehicle is moving
        Fx_FL *= (((this->torques.FL) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        Fx_FR *= (((this->torques.FR) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        Fx_RL *= (((this->torques.RL) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        Fx_RR *= (((this->torques.RR) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        
        // Calculate tire side slip angles
        double eps = 0.00001;

        double kappaFront = std::atan2(vFront.y(), std::max(std::abs(vFront.x()), eps)) - steeringFront;
        double kappaRear = std::atan2(vRear.y(), std::max(std::abs(vRear.x()), eps));

        if (stillstand)
        {
            kappaFront = 0.0;
            kappaRear = 0.0;
        }

        double Fy_Front = processSlipAngleLat(kappaFront, Fz_Front/2) * 2;
        double Fy_Rear = processSlipAngleLat(kappaRear, Fz_Rear/2) * 2;

        // Convert wheel speeds to RPMr
        this->wheelspeeds.FL = vFL.x() / rpm2ms;
        this->wheelspeeds.FR = vFR.x() / rpm2ms;
        this->wheelspeeds.RL = vRL.x() / rpm2ms;
        this->wheelspeeds.RR = vRR.x() / rpm2ms;

        // Calculate longitudinal and lateral accelerations
        double axTires = (std::cos(this->steeringAngles.FL) * Fx_FL + std::cos(this->steeringAngles.FR) * Fx_FR + Fx_RL
                             + Fx_RR - std::sin(steeringFront) * Fy_Front)
            / m;
        double axModel = axTires - F_aero_drag / m - friction.x() / m;

        double ayTires = (std::sin(this->steeringAngles.FL) * Fx_FL + std::sin(this->steeringAngles.FR) * Fx_FR
                             + std::cos(steeringFront) * Fy_Front + Fy_Rear)
            / m;
        double ayModel = (ayTires);

        // Calculate the rate of change of yaw rate (angular acceleration)
        double rdotFx
            = 0.5 * this->sf * (-Fx_FL * std::cos(this->steeringAngles.FL) + Fx_FR * std::cos(this->steeringAngles.FR))
            + this->lf * (Fx_FL * std::sin(this->steeringAngles.FL) + Fx_FR * std::sin(this->steeringAngles.FR))
            + 0.5 * this->sr * (Fx_RR - Fx_RL);
        double rdotFy = this->lf * (Fy_Front * std::cos(steeringFront)) - this->lr * (Fy_Rear);
        double rdot = (1 / Izz * (rdotFx + rdotFy));

        // Return the calculated dynamic states
        Eigen::Vector3d ret(axModel, ayModel, rdot);
        return ret;
    }

    // Integrate the vehicle state forward in time by dt
    void forwardIntegrate(double dt)
    {
        // Update position based on velocity and orientation
        Eigen::AngleAxisd yawAngle(this->orientation.z(), Eigen::Vector3d::UnitZ());
        this->position += (yawAngle.matrix() * this->velocity) * dt;

        // Convert the throttle actuation to torques
        // Torque is half for each wheel, 120nm is max total torque.
        this->torques.FL = 0;
        this->torques.FR = 0;
        this->torques.RL = this->throttleActuation * 120 * 0.5;
        this->torques.RR = this->throttleActuation * 120 * 0.5;

        // Get dynamic states
        Eigen::Vector3d xdotdyn = getDynamicStates(dt);

        // Update orientation based on angular velocity
        this->orientation += Eigen::Vector3d(0.0, 0.0, dt * angularVelocity.z());

        // Update acceleration based on dynamic states and friction
        this->acceleration = Eigen::Vector3d(xdotdyn[0], xdotdyn[1], 0.0);

        // Update angular velocity and angular acceleration
        this->angularVelocity = (this->angularVelocity + Eigen::Vector3d(0.0, 0.0, xdotdyn[2] * dt));
        this->angularAcceleration = Eigen::Vector3d(0.0, 0.0, xdotdyn[2]);

        // Update velocity based on acceleration and angular velocity
        this->velocity += dt * (this->acceleration - this->angularVelocity.cross(this->velocity));

        // Check for very small velocities and explicitly set to zero
        if (this->velocity.norm() < 0.1) {
            this->velocity = Eigen::Vector3d::Zero();
        }

        // Update wheel orientations based on wheel speeds
        this->wheelOrientations.FL = std::fmod(
            this->wheelOrientations.FL + (this->wheelspeeds.FL / (60.0 * this->gearRatio)) * dt * 2.0 * M_PI,
            2.0 * M_PI);
        this->wheelOrientations.FR = std::fmod(
            this->wheelOrientations.FR + (this->wheelspeeds.FR / (60.0 * this->gearRatio)) * dt * 2.0 * M_PI,
            2.0 * M_PI);
        this->wheelOrientations.RL = std::fmod(
            this->wheelOrientations.RL + (this->wheelspeeds.RL / (60.0 * this->gearRatio)) * dt * 2.0 * M_PI,
            2.0 * M_PI);
        this->wheelOrientations.RR = std::fmod(
            this->wheelOrientations.RR + (this->wheelspeeds.RR / (60.0 * this->gearRatio)) * dt * 2.0 * M_PI,
            2.0 * M_PI);
    }

    std::array<Eigen::Vector3d, 4> getWheelPositions()
    {
        auto rotMat = eulerAnglesToRotMat(this->orientation).transpose();
        Eigen::Vector3d FL = rotMat * Eigen::Vector3d(this->lf, this->sf * 0.5, 0.0) + this->position;
        Eigen::Vector3d FR = rotMat * Eigen::Vector3d(this->lf, -this->sf * 0.5, 0.0) + this->position;
        Eigen::Vector3d RL = rotMat * Eigen::Vector3d(-this->lr, this->sr * 0.5, 0.0) + this->position;
        Eigen::Vector3d RR = rotMat * Eigen::Vector3d(-this->lr, -this->sr * 0.5, 0.0) + this->position;
        std::array<Eigen::Vector3d, 4> ret { FL, FR, RL, RR };
        return ret;
    }

private:
    // Vehicle parameters
    double lr; // Distance from the center of gravity to the rear axle
    double lf; // Distance from the center of gravity to the front axle
    double sf; // Track width front
    double sr; // Track width rear

    // Tire model parameters
    double Blat;
    double Clat;
    double Dlat;
    double Elat;

    // Aerodynamic parameters
    double cla;
    double cda;
    double aeroArea;

    // Vehicle mass and inertia
    double m;
    double Izz;

    // Wheel and steering parameters
    double wheelRadius;
    double gearRatio;
    double innerSteeringRatio;
    double outerSteeringRatio;
    double nominalVoltageTS;
    double powerGroundSetpoint = 0.0;
    double powerGroundForce;
    // double powertrainEfficiency = 1.0;

    // Wheel torques and speeds
    Wheels minTorques = { -0.0, -0.0, -0.0, -0.0 };
    Wheels maxTorques = { 0.0, 0.0, 0.0, 0.0 };
    Wheels rpmSetpoints = { 0.0, 0.0, 0.0, 0.0 };
    Wheels currentFx = { 0.0, 0.0, 0.0, 0.0 };
    Wheels currentFy = { 0.0, 0.0, 0.0, 0.0 };
    double throttleActuation = 0.0;
};
