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
  static int log_count = 0;
  // Motor
  double throttle_input =
      (throttle.rear_left + throttle.rear_right) / 2.0;  // Average throttle for rear-wheel drive
  double motor_torque = calculate_powertrain_torque(throttle_input, dt);

  // Distribute torque to the wheels
  state_->wheels_torque =
      differential_->calculateTorqueDistribution(motor_torque, state_->wheels_speed);

  if (log_count % 100 == 0) {
    RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"),
                "Aero parameters: frontal_area=%.2f, drag_coefficient=%.2f",
                simulator_parameters_->car_parameters->aero_parameters->frontal_area,
                simulator_parameters_->car_parameters->aero_parameters->drag_coefficient);
    RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"), "Velocities (m/s): vx=%.2f vy=%.2f",
                state_->vx, state_->vy);
  }
  // Aerodynamics
  // based on implementation, this forces are negative by default, so we add them
  Eigen::Vector3d aero_forces =
      aero_->aero_forces(Eigen::Vector3d(state_->vx, state_->vy, state_->yaw_rate));

  // Ackerman steering
  double R = simulator_parameters_->car_parameters->wheelbase / (tan(angle) + 1e-6);
  double af = simulator_parameters_->car_parameters->ackerman_factor;

  double actual_steering_fl =
      angle +
      af * (atan(simulator_parameters_->car_parameters->wheelbase /
                 (R - simulator_parameters_->car_parameters->track_width / 2.0)) -
            angle) +
      simulator_parameters_->car_parameters->tire_parameters->fl_toe;  // toe should be signed
  double actual_steering_fr =
      angle +
      af * (atan(simulator_parameters_->car_parameters->wheelbase /
                 (R + simulator_parameters_->car_parameters->track_width / 2.0)) -
            angle) +
      simulator_parameters_->car_parameters->tire_parameters->fr_toe;

  // Load Transfer
  common_lib::structures::Wheels load_on_wheels = load_transfer_->compute_loads(
      LoadTransferInput{state_->ax, state_->ay, aero_forces[2]});  // aero_forces{Fx,Fy,Fz}

  // Tire
  Eigen::Vector3d front_left_forces =
      calculateTireForces("fl", load_on_wheels.front_left, actual_steering_fl);
  Eigen::Vector3d front_right_forces =
      calculateTireForces("fr", load_on_wheels.front_right, actual_steering_fr);
  Eigen::Vector3d rear_left_forces = calculateTireForces(
      "rl", load_on_wheels.rear_left, 0.0);  // Rear wheels do not steer, so steering angle is 0
  Eigen::Vector3d rear_right_forces = calculateTireForces("rr", load_on_wheels.rear_right, 0.0);

  // Low-speed lateral force scaling
  double v_total = std::sqrt(state_->vx * state_->vx + state_->vy * state_->vy);
  double low_speed_factor = std::min(1.0, v_total / 1.0);  // fully active above 1 m/s

  front_left_forces[1] *= low_speed_factor;
  front_right_forces[1] *= low_speed_factor;
  rear_left_forces[1] *= low_speed_factor;
  rear_right_forces[1] *= low_speed_factor;
  front_left_forces[2] *= low_speed_factor;
  front_right_forces[2] *= low_speed_factor;
  rear_left_forces[2] *= low_speed_factor;
  rear_right_forces[2] *= low_speed_factor;

  // Update wheel speeds
  // Net torque = drive - tire_reaction (F * r acts as a braking moment on the wheel)
  state_->wheels_speed.rear_left +=
      ((state_->wheels_torque.rear_left -
        rear_left_forces[0] *
            simulator_parameters_->car_parameters->tire_parameters->effective_tire_r) /
       simulator_parameters_->car_parameters->tire_parameters->wheel_inertia) *
      dt;

  //   RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"),
  //               "Rear left force: %.2f, effective radius: %.2f, wheel inertia: %.2f",
  //               rear_left_forces[0],
  //               simulator_parameters_->car_parameters->tire_parameters->effective_tire_r,
  //               simulator_parameters_->car_parameters->tire_parameters->wheel_inertia);
  state_->wheels_speed.rear_right +=
      ((state_->wheels_torque.rear_right -
        rear_right_forces[0] *
            simulator_parameters_->car_parameters->tire_parameters->effective_tire_r) /
       simulator_parameters_->car_parameters->tire_parameters->wheel_inertia) *
      dt;
  // front wheels are unpowered, only tire reaction
  state_->wheels_speed.front_left +=
      ((-front_left_forces[0] *
        simulator_parameters_->car_parameters->tire_parameters->effective_tire_r) /
       simulator_parameters_->car_parameters->tire_parameters->wheel_inertia) *
      dt;
  state_->wheels_speed.front_right +=
      ((-front_right_forces[0] *
        simulator_parameters_->car_parameters->tire_parameters->effective_tire_r) /
       simulator_parameters_->car_parameters->tire_parameters->wheel_inertia) *
      dt;

  if (std::isnan(state_->wheels_speed.rear_right)) {
    exit(1);
  }
  if (log_count % 100 == 0) {
    RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"),
                "Wheel speeds (rad/s): FL=%.2f FR=%.2f RL=%.2f RR=%.2f",
                state_->wheels_speed.front_left, state_->wheels_speed.front_right,
                state_->wheels_speed.rear_left, state_->wheels_speed.rear_right);
    // RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"), "Steering angles (rad): FL=%.2f FR=%.2f",
    //             actual_steering_fl, actual_steering_fr);

    // RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"),
    //             "Load on wheels: FL=%.2f FR=%.2f RL=%.2f RR=%.2f", load_on_wheels.front_left,
    //             load_on_wheels.front_right, load_on_wheels.rear_left, load_on_wheels.rear_right);

    // RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"),
    //             "Tire forces (N): FL=(%.2f, %.2f, %.2f) FR=(%.2f, %.2f, %.2f) RL=(%.2f, %.2f, "
    //             "%.2f) RR=(%.2f, %.2f, %.2f)",
    //             front_left_forces[0], front_left_forces[1], front_left_forces[2],
    //             front_right_forces[0], front_right_forces[1], front_right_forces[2],
    //             rear_left_forces[0], rear_left_forces[1], rear_left_forces[2],
    //             rear_right_forces[0], rear_right_forces[1], rear_right_forces[2]);

    RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"), "Aero forces (N): Fx=%.2f Fy=%.2f Fz=%.2f",
                aero_forces[0], aero_forces[1], aero_forces[2]);
  }
  // Vehicle State Update
  // Sum of all forces normalized to the vehicle coordinate system
  double Fx_fl = front_left_forces[0] * cos(actual_steering_fl) -
                 front_left_forces[1] * sin(actual_steering_fl);
  double Fy_fl = front_left_forces[0] * sin(actual_steering_fl) +
                 front_left_forces[1] * cos(actual_steering_fl);
  double Fx_fr = front_right_forces[0] * cos(actual_steering_fr) -
                 front_right_forces[1] * sin(actual_steering_fr);
  double Fy_fr = front_right_forces[0] * sin(actual_steering_fr) +
                 front_right_forces[1] * cos(actual_steering_fr);
  double total_fx = Fx_fl + Fx_fr + rear_left_forces[0] + rear_right_forces[0] + aero_forces[0];
  double total_fy = Fy_fl + Fy_fr + rear_left_forces[1] + rear_right_forces[1] + aero_forces[1];

  // Update accelerations using low-pass filter
  double ax_unfiltered =
      total_fx / simulator_parameters_->car_parameters->total_mass + state_->vy * state_->yaw_rate;
  double ay_unfiltered =
      total_fy / simulator_parameters_->car_parameters->total_mass - state_->vx * state_->yaw_rate;

  state_->ax =
      ax_unfiltered * 0.3 + state_->ax * 0.7;  // Simple low-pass filter for smoother acceleration
  state_->ay =
      ay_unfiltered * 0.3 + state_->ay * 0.7;  // Simple low-pass filter for smoother acceleration

  // Update velocities
  state_->vx += state_->ax * dt;
  state_->vy += state_->ay * dt;

  double lr =
      simulator_parameters_->car_parameters->cg_2_rear_axis;  // Distance from CG to rear axle
  double lf =
      simulator_parameters_->car_parameters->wheelbase - lr;  // Distance from CG to front axle
  double half_width = simulator_parameters_->car_parameters->track_width / 2.0;

  // 1. Moment from Lateral Forces (Fy)
  double moment_fy = (Fy_fl + Fy_fr) * lf - (rear_left_forces[1] + rear_right_forces[1]) * lr;

  // 2. Moment from Longitudinal Forces (Fx)
  // Right side pushing forward (+) and Left side pushing forward (-) creates yaw
  //   double moment_fx =
  //       (Fx_fr + rear_right_forces[0]) * half_width - (Fx_fl + rear_left_forces[0]) *
  //       half_width;
  double moment_fx = (Fx_fr - Fx_fl) * half_width                                  // front axle
                     + (rear_right_forces[0] - rear_left_forces[0]) * half_width;  // rear axle

  // 3. Self-Aligning Moments (Mz) from the tires themselves
  double total_mz =
      front_left_forces[2] + front_right_forces[2] + rear_left_forces[2] + rear_right_forces[2];

  double total_torque = moment_fy + moment_fx + total_mz;

  // Update yaw
  double yaw_a = total_torque / simulator_parameters_->car_parameters->Izz;
  state_->yaw_rate += yaw_a * dt;

  state_->yaw += state_->yaw_rate * dt;

  // Keep yaw within [-pi, pi]
  if (state_->yaw > M_PI) state_->yaw -= 2.0 * M_PI;
  if (state_->yaw < -M_PI) state_->yaw += 2.0 * M_PI;

  // Update X and Y positions
  // 1. Calculate Global Velocities
  double cos_yaw = cos(state_->yaw);
  double sin_yaw = sin(state_->yaw);

  double v_global_x = state_->vx * cos_yaw - state_->vy * sin_yaw;
  double v_global_y = state_->vx * sin_yaw + state_->vy * cos_yaw;

  // 2. Update Global Positions (Integration)
  state_->x += v_global_x * dt;
  state_->y += v_global_y * dt;

  // Save the tire forces for the next step's wheel speed update
  state_->front_left_forces = front_left_forces;
  state_->front_right_forces = front_right_forces;
  state_->rear_left_forces = rear_left_forces;
  state_->rear_right_forces = rear_right_forces;

  log_count++;
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
  state_->ax = 0.0;
  state_->ay = 0.0;
  state_->yaw_rate = 0.0;
  state_->wheels_speed = {0.0, 0.0, 0.0, 0.0};
  state_->wheels_torque = {0.0, 0.0, 0.0, 0.0};
  state_->front_left_forces = {0.0, 0.0, 0.0};
  state_->front_right_forces = {0.0, 0.0, 0.0};
  state_->rear_left_forces = {0.0, 0.0, 0.0};
  state_->rear_right_forces = {0.0, 0.0, 0.0};
}

std::string FSFEUP02Model::get_model_name() const { return "FSFEUP02Model"; }

double FSFEUP02Model::get_motor_torque() const { return motor_->get_torque(); }

double FSFEUP02Model::get_battery_current() const { return battery_->get_current(); }

double FSFEUP02Model::get_battery_voltage() const { return battery_->get_voltage(); }

double FSFEUP02Model::get_battery_soc() const { return battery_->get_soc(); }

double FSFEUP02Model::calculate_powertrain_torque(double throttle_input, double dt) {
  // 1. Estados Físicos Atualizados
  float avg_wheel_speed = (state_->wheels_speed.rear_left + state_->wheels_speed.rear_right) / 2.0f;
  float motor_omega = avg_wheel_speed * simulator_parameters_->car_parameters->gear_ratio;
  float motor_rpm = (motor_omega * 60.0f / (2.0f * static_cast<float>(M_PI)));

  // 2. Torque de Referência (O máximo que o motor/piloto deseja)
  float max_motor_torque = motor_->get_max_torque_at_rpm(motor_rpm);
  float reference_torque = static_cast<float>(throttle_input) * max_motor_torque;

  // 3. Eficiência e Constante de Torque
  float efficiency = motor_->get_efficiency(std::abs(reference_torque), motor_rpm);
  float kt = simulator_parameters_->car_parameters->motor_parameters->kt_constant;

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

double FSFEUP02Model::calculateSlipAngleFront(double dist_to_cg, bool isLeft,
                                              double steering_angle) {
  const double V_eps = 1;  // Regularization constant to prevent incorrect low speed behavior can
                           // be lower if needed
  double sign = (isLeft) ? -1.0 : 1.0;  // Sign used to apply the effect of yaw_rate
  // Normalize velocity to wheels coordinate system
  double Vcx = state_->vx * cos(steering_angle) + state_->vy * sin(steering_angle);
  double Vcy = -state_->vx * sin(steering_angle) + state_->vy * cos(steering_angle);
  return (
      atan((Vcy + (state_->yaw_rate * dist_to_cg) +
            (sign * state_->yaw_rate * simulator_parameters_->car_parameters->track_width / 2)) /
           sqrt(Vcx * Vcx + V_eps * V_eps)));
}

double FSFEUP02Model::calculateSlipAngleRear(double dist_to_cg, bool isLeft) {
  const double V_eps = 1;
  double sign = (isLeft) ? -1.0 : 1.0;

  // Lateral velocity at the wheel contact patch
  double Vcy_contact =
      state_->vy - (state_->yaw_rate * dist_to_cg) +
      (sign * state_->yaw_rate * simulator_parameters_->car_parameters->track_width / 2.0);

  return atan(Vcy_contact / sqrt(state_->vx * state_->vx + V_eps * V_eps));
}

double FSFEUP02Model::calculateSlipRatio(double wheel_angular_velocity,
                                         double effective_tire_radius, double steering_angle) {
  const double V_eps = 1;  // Regularization constant to prevent incorrect low speed behavior can
                           // be lower if needed
  // Normalize velocity to wheels coordinate system
  double Vcx = state_->vx * cos(steering_angle) + state_->vy * sin(steering_angle);
  double slip_ratio =
      (wheel_angular_velocity * effective_tire_radius - Vcx) / sqrt(Vcx * Vcx + V_eps * V_eps);
  slip_ratio = std::clamp(slip_ratio, -1.0, 1.0);  // hard physical limit
  return slip_ratio;
}

Eigen::Vector3d FSFEUP02Model::calculateTireForces(std::string tire_name, double load,
                                                   double steering_angle) {
  TireInput tire_input;
  tire_input.vx = state_->vx;
  tire_input.vy = state_->vy;
  tire_input.yaw_rate = state_->yaw_rate;
  tire_input.steering_angle = steering_angle;
  tire_input.vertical_load = load;
  if (tire_name == "fl") {
    tire_input.distance_to_CG = simulator_parameters_->car_parameters->tire_parameters->d_fleft;
    tire_input.camber_angle = simulator_parameters_->car_parameters->tire_parameters->fl_camber;
    tire_input.slip_angle = calculateSlipAngleFront(
        simulator_parameters_->car_parameters->tire_parameters->d_fleft, true, steering_angle);
    tire_input.slip_ratio =
        calculateSlipRatio(state_->wheels_speed.front_left,
                           simulator_parameters_->car_parameters->tire_parameters->effective_tire_r,
                           steering_angle);  // Assuming wheel speed is in rad/s
    tire_input.wheel_angular_speed = state_->wheels_speed.front_left;
    return front_left->tire_forces(tire_input);
  } else if (tire_name == "fr") {
    tire_input.distance_to_CG = simulator_parameters_->car_parameters->tire_parameters->d_fright;
    tire_input.camber_angle = simulator_parameters_->car_parameters->tire_parameters->fr_camber;
    tire_input.slip_angle = calculateSlipAngleFront(
        simulator_parameters_->car_parameters->tire_parameters->d_fright, false, steering_angle);
    tire_input.slip_ratio =
        calculateSlipRatio(state_->wheels_speed.front_right,
                           simulator_parameters_->car_parameters->tire_parameters->effective_tire_r,
                           steering_angle);  // Assuming wheel speed is in rad/s
    tire_input.wheel_angular_speed = state_->wheels_speed.front_right;
    return front_right->tire_forces(tire_input);
  } else if (tire_name == "rl") {
    tire_input.distance_to_CG = simulator_parameters_->car_parameters->tire_parameters->d_bleft;
    tire_input.camber_angle = simulator_parameters_->car_parameters->tire_parameters->rl_camber;
    tire_input.slip_angle = calculateSlipAngleRear(
        simulator_parameters_->car_parameters->tire_parameters->d_bleft, true);
    // RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"), "Slip angle RL (rad): %.2f",
    //             tire_input.slip_angle);
    tire_input.slip_ratio =
        calculateSlipRatio(state_->wheels_speed.rear_left,
                           simulator_parameters_->car_parameters->tire_parameters->effective_tire_r,
                           steering_angle);  // Assuming wheel speed is in rad/s
    tire_input.wheel_angular_speed = state_->wheels_speed.rear_left;
    return rear_left->tire_forces(tire_input);
  } else if (tire_name == "rr") {
    tire_input.distance_to_CG = simulator_parameters_->car_parameters->tire_parameters->d_bright;
    tire_input.camber_angle = simulator_parameters_->car_parameters->tire_parameters->rr_camber;
    tire_input.slip_angle = calculateSlipAngleRear(
        simulator_parameters_->car_parameters->tire_parameters->d_bright, false);
    // RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"), "Slip angle RR (rad): %.2f",
    //             tire_input.slip_angle);
    tire_input.slip_ratio =
        calculateSlipRatio(state_->wheels_speed.rear_right,
                           simulator_parameters_->car_parameters->tire_parameters->effective_tire_r,
                           steering_angle);  // Assuming wheel speed is in rad/s
    tire_input.wheel_angular_speed = state_->wheels_speed.rear_right;
    return rear_right->tire_forces(tire_input);
  } else {
    throw std::invalid_argument("Invalid tire name");
  }
}