#pragma once

#include <memory>
#include <cmath>
#include <Eigen/Dense>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/structures/wheels.hpp"

#include "motion_lib/aero_model/map.hpp"
#include "motion_lib/battery_model/map.hpp"
#include "motion_lib/brake_model/map.hpp"
#include "motion_lib/inverter_model/map.hpp"
#include "motion_lib/load_transfer_model/map.hpp"
#include "motion_lib/motor_model/map.hpp"
#include "motion_lib/steering_model/map.hpp"
#include "motion_lib/steering_motor_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"
#include "motion_lib/transmission_model/map.hpp"

struct SimState {
    double vx = 0.0, vy = 0.0, yaw_rate = 0.0, yaw = 0.0, x = 0.0, y = 0.0;
    double steering_angle = 0.0, ax = 0.0, ay = 0.0;
    
    common_lib::structures::Wheels wheels_speed;
    
    double motor_torque = 0.0, motor_omega = 0.0, motor_current = 0.0;
    double motor_thermal_state = 0.0, motor_thermal_capacity = 0.0;
    double battery_current = 0.0, battery_voltage = 0.0, battery_soc = 0.0, battery_open_circuit_voltage = 0.0;

    common_lib::structures::Wheels wheels_slip_ratio;
    common_lib::structures::Wheels wheels_slip_angle;
    common_lib::structures::Wheels wheels_torque;
    common_lib::structures::Wheels wheels_vertical_load;

    bool ebs_active = false;
};

class InlineFSFEUP02Model {
private:
    std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;
    std::shared_ptr<TireModel> tire_model_;
    std::shared_ptr<MotorModel> motor_;
    std::shared_ptr<BatteryModel> battery_;
    std::shared_ptr<TransmissionModel> transmission_;
    std::shared_ptr<InverterModel> inverter_;
    std::shared_ptr<BrakeModel> brake_;
    std::shared_ptr<AeroModel> aero_;
    std::shared_ptr<LoadTransferModel> load_transfer_;
    std::shared_ptr<SteeringModel> steering_;
    std::shared_ptr<SteeringMotorModel> steering_motor_;
    std::string control_mode_;

    // Pre-computed constants to replace slow division with fast multiplication
    double inv_mass_;
    double inv_izz_;
    double inv_wheel_inertia_;
    double half_track_width_;

    enum StateIndex {
        VX = 0, VY, YAW_RATE, YAW, PX, PY, ST_ANGLE,
        FL_W, FR_W, RL_W, RR_W, AX, AY
    };
    using StateVec = Eigen::Matrix<double, 13, 1>;

    inline double calculate_powertrain_torque(double throttle_input, double dt, SimState& state) {
        double motor_omega = transmission_->calculate_motor_omega(state.wheels_speed);
        double motor_rpm = std::abs(motor_omega * 60.0 / (2.0 * M_PI));

        double max_motor_torque = motor_->get_max_torque_at_rpm(motor_rpm);
        double reference_motor_torque = throttle_input * max_motor_torque;

        const double min_omega_for_power = 10.0; 
        double omega_sign_source = std::abs(motor_omega) > 1e-3 ? motor_omega : reference_motor_torque;
        double omega_for_power = std::copysign(std::max(std::abs(motor_omega), min_omega_for_power), omega_sign_source);

        double mechanical_power_request = reference_motor_torque * omega_for_power;
        double motor_efficiency = motor_->get_efficiency(std::abs(reference_motor_torque), motor_rpm);
        double inverter_efficiency = inverter_->get_efficiency();
        double total_efficiency = motor_efficiency * inverter_efficiency;

        double battery_power_request = (mechanical_power_request >= 0.0) 
            ? (mechanical_power_request / total_efficiency) 
            : (mechanical_power_request * total_efficiency);

        double battery_voltage = battery_->get_voltage();
        double requested_battery_current = battery_power_request / battery_voltage;
        double allowed_battery_current = battery_->calculate_allowed_current(requested_battery_current);

        double allowed_battery_voltage = battery_->get_voltage(allowed_battery_current);
        double allowed_battery_power = allowed_battery_current * allowed_battery_voltage;

        double allowed_mechanical_power = (allowed_battery_power >= 0.0)
            ? (allowed_battery_power * total_efficiency)
            : (allowed_battery_power / total_efficiency);

        double actual_motor_torque = std::copysign(
            std::min(std::abs(allowed_mechanical_power / omega_for_power), std::abs(reference_motor_torque)),
            reference_motor_torque);

        double motor_phase_current = std::abs(actual_motor_torque) / car_parameters_->motor_parameters->kt_constant;
        double max_inverter_phase_current = car_parameters_->inverter_parameters->max_phase_current;
        
        if (motor_phase_current > max_inverter_phase_current) {
            actual_motor_torque = std::copysign(max_inverter_phase_current * car_parameters_->motor_parameters->kt_constant, actual_motor_torque);
            motor_phase_current = max_inverter_phase_current;
        }

        const double signed_motor_phase_current = std::abs(allowed_battery_current) > 1e-9
            ? std::copysign(motor_phase_current, allowed_battery_current) : 0.0;

        battery_->update_state(allowed_battery_current, dt);
        motor_->update_state(signed_motor_phase_current, actual_motor_torque, dt);

        state.motor_torque = actual_motor_torque;
        state.motor_omega = motor_omega;
        state.motor_current = motor_->get_current();
        state.motor_thermal_state = motor_->get_thermal_state();
        state.motor_thermal_capacity = motor_->get_thermal_capacity();
        state.battery_current = battery_->get_current();
        state.battery_voltage = battery_->get_voltage();
        state.battery_soc = battery_->get_soc();
        state.battery_open_circuit_voltage = battery_->get_open_circuit_voltage();

        return actual_motor_torque;
    }

    // Passed ds by reference to avoid copy overhead. Used fixed size Matrix for tire_forces to avoid heap allocation.
    inline void get_state_derivative(
        const StateVec& s, double motor_torque, const common_lib::structures::Wheels& brake_torques,
        double steering_target, double dt, SimState& state,
        Eigen::Vector4d& out_slip_ratio, Eigen::Vector4d& out_slip_angle, StateVec& ds) {
        
        ds.setZero();
        const common_lib::structures::Wheels wheel_speeds(s(FL_W), s(FR_W), s(RL_W), s(RR_W));
        const common_lib::structures::Wheels wheel_torques = transmission_->calculate_wheel_torques(motor_torque, wheel_speeds);
        const Eigen::Vector4d torques(wheel_torques.front_left, wheel_torques.front_right, wheel_torques.rear_left, wheel_torques.rear_right);

        Eigen::Vector4d wheel_angles = steering_->calculate_steering_angles(s(ST_ANGLE));
        const Eigen::Vector3d aero_forces = aero_->aero_forces(Eigen::Vector3d(s(VX), s(VY), s(YAW_RATE)));
        const common_lib::structures::Wheels load_distribution = load_transfer_->compute_loads(LoadTransferInput{s(AX), s(AY), aero_forces[2]}); 
        const Eigen::Vector4d vertical_loads(load_distribution.front_left, load_distribution.front_right, load_distribution.rear_left, load_distribution.rear_right);

        // STACK ALLOCATION (Fast): Replaced dynamic VectorXd(16) with fixed-size array
        Eigen::Matrix<double, 16, 1> tire_forces;
        Eigen::Vector4d contact_patch_longitudinal_velocity = Eigen::Vector4d::Zero();
        
        TireInput tire_input;
        tire_input.dt = dt;
        tire_input.vx = s(VX);
        tire_input.vy = s(VY);
        tire_input.yaw_rate = s(YAW_RATE);
        tire_input.last_slip_ratio << state.wheels_slip_ratio.front_left, state.wheels_slip_ratio.front_right, state.wheels_slip_ratio.rear_left, state.wheels_slip_ratio.rear_right;
        tire_input.last_slip_angle << state.wheels_slip_angle.front_left, state.wheels_slip_angle.front_right, state.wheels_slip_angle.rear_left, state.wheels_slip_angle.rear_right;

        for (Tire tire : {FL, FR, RL, RR}) {
            tire_input.tire = tire;
            tire_input.steering_angle = wheel_angles(tire);
            tire_input.wheel_angular_speed = s(FL_W + tire);
            tire_input.vertical_load = vertical_loads(tire);
            tire_forces.segment<4>(tire * 4) = tire_model_->calculate_tire_forces(tire_input);
            out_slip_ratio(tire) = tire_input.slip_ratio;
            out_slip_angle(tire) = tire_input.slip_angle;
            contact_patch_longitudinal_velocity(tire) = tire_input.vcx;
        }

        double total_fx = aero_forces[0], total_fy = aero_forces[1], total_torque = 0.0;
        const double lr = car_parameters_->cg_2_rear_axis;
        const double lf = car_parameters_->wheelbase - lr;
        const double wheel_radius = car_parameters_->tire_parameters->effective_tire_r;
        const Eigen::Vector4d brake_torques_by_tire(brake_torques.front_left, brake_torques.front_right, brake_torques.rear_left, brake_torques.rear_right);

        for (Tire tire : {FL, FR, RL, RR}) {
            double fx_tire = tire_forces(tire * 4);
            const double fy_tire = tire_forces(tire * 4 + 1);
            const double mz_tire = tire_forces(tire * 4 + 3);
            const double cos_delta = std::cos(wheel_angles(tire));
            const double sin_delta = std::sin(wheel_angles(tire));
            const double brake_torque = brake_torques_by_tire(tire);
            
            if (brake_torque > 0.0) {
                const double brake_sign = 2.0 / M_PI * std::atan(10.0 * contact_patch_longitudinal_velocity(tire));
                fx_tire -= brake_torque * brake_sign / std::max(wheel_radius, 1e-6);
            }

            const double fx_vehicle = fx_tire * cos_delta - fy_tire * sin_delta;
            const double fy_vehicle = fx_tire * sin_delta + fy_tire * cos_delta;
            const double arm_x = (tire == FL || tire == FR) ? lf : -lr;
            const double arm_y = (tire == FL || tire == RL) ? half_track_width_ : -half_track_width_;

            total_fx += fx_vehicle;
            total_fy += fy_vehicle;
            total_torque += arm_x * fy_vehicle - arm_y * fx_vehicle + mz_tire;
        }

        // Fast multiplication instead of division
        const double ax = total_fx * inv_mass_ + s(VY) * s(YAW_RATE);
        const double ay = total_fy * inv_mass_ - s(VX) * s(YAW_RATE);
        
        ds(VX) = ax; ds(VY) = ay; ds(AX) = ax - s(AX); ds(AY) = ay - s(AY);
        ds(YAW_RATE) = total_torque * inv_izz_; ds(YAW) = s(YAW_RATE);
        ds(PX) = s(VX) * std::cos(s(YAW)) - s(VY) * std::sin(s(YAW));
        ds(PY) = s(VX) * std::sin(s(YAW)) + s(VY) * std::cos(s(YAW));
        ds(ST_ANGLE) = steering_motor_->compute_steering_rate(s(ST_ANGLE), steering_target);

        if (state.ebs_active) {
            double inv_dt_clamped = 1.0 / std::max(dt, 1e-6);
            ds(FL_W) = -s(FL_W) * inv_dt_clamped;
            ds(FR_W) = -s(FR_W) * inv_dt_clamped;
            ds(RL_W) = -s(RL_W) * inv_dt_clamped;
            ds(RR_W) = -s(RR_W) * inv_dt_clamped;
        } else {
            for (Tire tire : {FL, FR, RL, RR}) {
                const double wheel_omega = s(FL_W + tire);
                double net_torque = torques(tire) - tire_forces(tire * 4) * wheel_radius - tire_forces(tire * 4 + 2);
                
                const double brake_sign = 2.0 / M_PI * std::atan(10.0 * wheel_omega);
                const double brake_torque = brake_torques_by_tire(tire);
                net_torque -= brake_torque * brake_sign;

                if (tire == FL || tire == FR) {
                    net_torque -= car_parameters_->front_bearing_drag * wheel_omega;
                }

                // Fast multiplication instead of division
                const double wheel_acceleration = net_torque * inv_wheel_inertia_;
                if (brake_torque > 0.0 && std::abs(wheel_omega) < 0.5 && wheel_acceleration * wheel_omega <= 0.0) {
                    ds(FL_W + tire) = -wheel_omega / std::max(dt, 1e-6);
                } else {
                    ds(FL_W + tire) = wheel_acceleration;
                }
            }
        }
    }

public:
    InlineFSFEUP02Model(const InvictaSimParameters& simulator_parameters) {
        car_parameters_ = simulator_parameters.car_parameters;
        tire_model_ = tire_models_map.at(simulator_parameters.tire_model.c_str())(simulator_parameters.car_parameters);
        motor_ = motor_models_map.at(simulator_parameters.motor_model.c_str())(simulator_parameters.car_parameters);
        battery_ = battery_models_map.at(simulator_parameters.battery_model.c_str())(simulator_parameters.car_parameters);
        transmission_ = transmission_models_map.at(simulator_parameters.transmission_model.c_str())(simulator_parameters.car_parameters);
        inverter_ = inverter_models_map.at(simulator_parameters.inverter_model.c_str())(simulator_parameters.car_parameters);
        brake_ = brake_models_map.at(simulator_parameters.brake_model.c_str())(simulator_parameters.car_parameters);
        aero_ = aero_models_map.at(simulator_parameters.aero_model.c_str())(simulator_parameters.car_parameters);
        load_transfer_ = load_transfer_models_map.at(simulator_parameters.load_transfer_model.c_str())(simulator_parameters.car_parameters);
        steering_ = steering_models_map.at(simulator_parameters.steering_model.c_str())(simulator_parameters.car_parameters);
        steering_motor_ = steering_motor_models_map.at(simulator_parameters.steering_motor_model.c_str())(simulator_parameters.car_parameters);
        control_mode_ = simulator_parameters.control_mode;

        // Cache inversions for blazing fast physics math
        inv_mass_ = 1.0 / car_parameters_->total_mass;
        inv_izz_ = 1.0 / car_parameters_->Izz;
        inv_wheel_inertia_ = 1.0 / car_parameters_->tire_parameters->wheel_inertia;
        half_track_width_ = car_parameters_->track_width / 2.0;
    }

    void step(double dt, const common_lib::structures::Wheels& throttle, double angle, SimState& state) {
        const double throttle_input = (throttle.rear_left + throttle.rear_right) / 2.0;
        common_lib::structures::Wheels brake_torques;
        double inverter_command = throttle_input;
        
        if (control_mode_ == "manual" && throttle_input < 0.0) {
            brake_torques = brake_->calculate_brake_torques(-throttle_input);
            inverter_command = 0.0;
        }

        const bool braking = brake_torques.front_left > 0.0 || brake_torques.front_right > 0.0 ||
                             brake_torques.rear_left > 0.0 || brake_torques.rear_right > 0.0;
                             
        const double motor_input = inverter_->calculate_inverter_throttle(inverter_command, dt);
        const double motor_torque = calculate_powertrain_torque(motor_input, dt, state);

        StateVec s;
        s << state.vx, state.vy, state.yaw_rate, state.yaw, state.x, state.y, state.steering_angle,
             state.wheels_speed.front_left, state.wheels_speed.front_right, state.wheels_speed.rear_left, state.wheels_speed.rear_right,
             state.ax, state.ay;

        Eigen::Vector4d k1_r, k1_a, k2_r, k2_a, k3_r, k3_a, k4_r, k4_a;
        StateVec k1, k2, k3, k4;

        get_state_derivative(s, motor_torque, brake_torques, angle, dt, state, k1_r, k1_a, k1);
        get_state_derivative(s + 0.5 * dt * k1, motor_torque, brake_torques, angle, dt, state, k2_r, k2_a, k2);
        get_state_derivative(s + 0.5 * dt * k2, motor_torque, brake_torques, angle, dt, state, k3_r, k3_a, k3);
        get_state_derivative(s + dt * k3, motor_torque, brake_torques, angle, dt, state, k4_r, k4_a, k4);
        
        StateVec s_next = s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

        state.vx = s_next(VX); state.vy = s_next(VY); state.yaw_rate = s_next(YAW_RATE);
        state.yaw = s_next(YAW); state.x = s_next(PX); state.y = s_next(PY); state.steering_angle = s_next(ST_ANGLE);
        state.wheels_speed = common_lib::structures::Wheels(s_next(FL_W), s_next(FR_W), s_next(RL_W), s_next(RR_W));
        state.ax = s_next(AX); state.ay = s_next(AY);

        if (state.ebs_active) {
            state.wheels_speed = {0.0, 0.0, 0.0, 0.0};
        }

        double speed = std::sqrt(state.vx * state.vx + state.vy * state.vy);
        if (speed < 0.05 && (std::abs(throttle_input) < 0.01 || braking)) {
            state.vx = 0.0; state.vy = 0.0; state.ax = 0.0; state.ay = 0.0; state.yaw_rate = 0.0;
            state.wheels_speed = {0.0, 0.0, 0.0, 0.0};
            state.wheels_slip_ratio = {0.0, 0.0, 0.0, 0.0};
            state.wheels_slip_angle = {0.0, 0.0, 0.0, 0.0};
        } else {
            state.wheels_slip_ratio = common_lib::structures::Wheels(k1_r(FL), k1_r(FR), k1_r(RL), k1_r(RR));
            state.wheels_slip_angle = common_lib::structures::Wheels(k1_a(FL), k1_a(FR), k1_a(RL), k1_a(RR));
        }

        if (state.yaw > M_PI) state.yaw -= 2.0 * M_PI;
        if (state.yaw < -M_PI) state.yaw += 2.0 * M_PI;
    }
};