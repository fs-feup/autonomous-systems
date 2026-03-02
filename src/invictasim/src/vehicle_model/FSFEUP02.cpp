#include "vehicle_model/FSFEUP02.hpp"

FSFEUP02Model::FSFEUP02Model(const InvictaSimParameters& simulator_parameters)
    : VehicleModel(simulator_parameters) {
  this->front_left = tire_models_map.at(simulator_parameters.tire_model.c_str())(
      simulator_parameters.car_parameters);
  this->front_right = tire_models_map.at(simulator_parameters.tire_model.c_str())(
      simulator_parameters.car_parameters);
  this->rear_left = tire_models_map.at(simulator_parameters.tire_model.c_str())(
      simulator_parameters.car_parameters);
  this->rear_right = tire_models_map.at(simulator_parameters.tire_model.c_str())(
      simulator_parameters.car_parameters);
  this->motor_ = motor_models_map.at(simulator_parameters.motor_model.c_str())(
      simulator_parameters.car_parameters);
  this->battery_ = battery_models_map.at(simulator_parameters.battery_model.c_str())(
      simulator_parameters.car_parameters);
  this->differential_ = differential_models_map.at(simulator_parameters.differential_model.c_str())(
      simulator_parameters.car_parameters);
  this->aero_ = aero_models_map.at(simulator_parameters.aero_model.c_str())(
      simulator_parameters.car_parameters);
  this->load_transfer_ = load_transfer_models_map.at(
      simulator_parameters.load_transfer_model.c_str())(simulator_parameters.car_parameters);
}

void FSFEUP02Model::step(double dt, common_lib::structures::Wheels throttle, double angle) {
  // 1. Average throttle input for rear-wheel drive logic
  double throttle_input = (throttle.rear_left + throttle.rear_right) / 2.0;

  // 2. Calculate torque (this now uses state_->vx to determine motor RPM)
  double motor_torque = calculate_powertrain_torque(throttle_input, dt);

  double mass = simulator_parameters_->car_parameters.total_mass;
  double gear_ratio = simulator_parameters_->car_parameters.gear_ratio;
  double wheel_radius = simulator_parameters_->car_parameters.wheel_diameter / 2.0;

  double tractive_force = (motor_torque * gear_ratio) / wheel_radius;

  // Basic Resistance Force (Rolling Resistance + Aero Drag)
  // Crr is typically ~0.015 for race tires [cite: 16]
  double rolling_resistance = 0.05 * mass * 9.81;
  double drag_force = 0.5 * 1.225 * 1.2 * 1.0 * std::pow(state_->vx, 2);

  // Net force determines acceleration
  double net_force = tractive_force;
  if (state_->vx > 0.1) {
    net_force -= (rolling_resistance + drag_force);
  } else if (std::abs(tractive_force) < rolling_resistance) {
    // Static friction: if torque is too low, don't move
    net_force = 0.0;
  }

  double acceleration = net_force / mass;

  // 4. Linear Integration of Velocity (v = v0 + a*dt)
  state_->vx += acceleration * dt;  // Prevent negative velocity (no reverse logic)

  // Safety clamp: prevent negative velocity
  if (state_->vx < 0.0) state_->vx = 0.0;

  // 5. Update Wheel Speeds for the NEXT simulation step
  // motor_rpm in calculate_powertrain_torque relies on this
  double wheel_omega = state_->vx / wheel_radius;

  state_->wheels_speed.rear_left = wheel_omega;
  state_->wheels_speed.rear_right = wheel_omega;
  state_->wheels_speed.front_left = wheel_omega;
  state_->wheels_speed.front_right = wheel_omega;

  // 6. Distribute torque to the wheels for state logging
  state_->wheels_torque =
      differential_->calculateTorqueDistribution(motor_torque, state_->wheels_speed);
}

void FSFEUP02Model::reset() {
  state_->x = 0.0;
  state_->y = 0.0;
  state_->z = 0.0;
  state_->roll = 0.0;
  state_->pitch = 0.0;
  state_->yaw = 0.0;
  state_->vx = 0.0;
  state_->vy = 0.0;
  state_->vz = 0.0;
}

std::string FSFEUP02Model::get_model_name() const { return "FSFEUP02Model"; }

double FSFEUP02Model::get_motor_torque() const { return motor_->get_torque(); }

double FSFEUP02Model::get_battery_current() const { return battery_->get_current(); }

double FSFEUP02Model::get_battery_voltage() const { return battery_->get_voltage(); }

double FSFEUP02Model::get_battery_soc() const { return battery_->get_soc(); }

double FSFEUP02Model::calculate_powertrain_torque(double throttle_input, double dt) {
  // 1. Estados Físicos Atualizados
  float avg_wheel_speed = (state_->wheels_speed.rear_left + state_->wheels_speed.rear_right) / 2.0f;
  float motor_omega = avg_wheel_speed * simulator_parameters_->car_parameters.gear_ratio;
  float motor_rpm = (motor_omega * 60.0f / (2.0f * static_cast<float>(M_PI)));

  // 2. Torque de Referência (O máximo que o motor/piloto deseja)
  float max_motor_torque = motor_->get_max_torque_at_rpm(motor_rpm);
  float reference_torque = static_cast<float>(throttle_input) * max_motor_torque;

  // 3. Eficiência e Constante de Torque
  float efficiency = motor_->get_efficiency(std::abs(reference_torque), motor_rpm);
  float kt = simulator_parameters_->car_parameters.motor_parameters->kt_constant;

  // 4. Conversão para Corrente de Referência (I = T / (Kt * eff))
  // Esta é a corrente necessária para atingir o torque do pedal.
  float requested_current = std::abs(reference_torque) / (kt * std::max(efficiency, 0.05f));

  // 5. O FILTRO DA BATERIA (calculate_allowed_current)
  // Esta função deve retornar o MIN(requested_current, limite_80kW, limite_Vmin, limite_Hardware)
  // Se a tensão cai, o limite_80kW sobe, mas o std::min selecionará o requested_current original.
  float battery_current = battery_->calculate_allowed_current(requested_current);

  // 6. Torque Real Resultante (T = I_real * Kt * eff)
  // Agora, se a tensão cair, a corrente NÃO sobe; ela fica travada no requested_current.
  float actual_motor_torque = battery_current * kt * efficiency;

  // 7. Correção de Direção e Atualização
  if (reference_torque < 0) actual_motor_torque *= -1.0f;

  battery_->update_state(battery_current, dt);
  motor_->update_state(battery_current, actual_motor_torque, dt);

  return static_cast<double>(actual_motor_torque);
}
