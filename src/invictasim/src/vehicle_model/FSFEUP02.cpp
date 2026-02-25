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
  double throttle_input =
      (throttle.rear_left + throttle.rear_right) / 2.0;  // Average throttle for rear-wheel drive
  double motor_torque = calculate_powertrain_torque(throttle_input, dt);
  state_->wheels_torque =
      differential_->calculateTorqueDistribution(motor_torque, state_->wheels_speed);
// Aerodynamics 
  //baesd on implementation, this forces are negative by default, so we add them
  Eigen::Vector3d aero_forces = aero_->aero_forces(Eigen::Vector3d(state_->vx, state_->vy, state_->angular_velocity));
  // Update accelerations based on aero
  state_->ax = (state_->ax + aero_forces[0]) / simulator_parameters_->car_parameters->total_mass;
  state_->ay = (state_->ay + aero_forces[1]) / simulator_parameters_->car_parameters->total_mass;
// Load Transfer
  common_lib::structures::Wheels load_on_wheels = load_transfer_->compute_loads(LoadTransferInput{state_->ax, state_->ay, aero_forces[2]});//aero_forces{Fx,Fy,Fz}
// Tire
  Eigen::Vector3d front_left_forces =  calculateTireForces("front_left", load_on_wheels.front_left);
  Eigen::Vector3d front_right_forces = calculateTireForces("front_right", load_on_wheels.front_right);
  Eigen::Vector3d rear_left_forces = calculateTireForces("rear_left", load_on_wheels.rear_left);
  Eigen::Vector3d rear_right_forces = calculateTireForces("rear_right", load_on_wheels.rear_right);
// Vehicle State Update
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
  state_->angular_velocity = 0.0;
  state_->ax = 0.0;
  state_->ay = 0.0;
}

std::string FSFEUP02Model::get_model_name() const { return "FSFEUP02Model"; }

double FSFEUP02Model::get_motor_torque() const { return motor_->get_torque(); }

double FSFEUP02Model::get_battery_current() const { return battery_->getCurrent(); }

double FSFEUP02Model::get_battery_voltage() const { return battery_->getVoltage(); }

double FSFEUP02Model::get_battery_soc() const { return battery_->getSoC(); }

double FSFEUP02Model::calculate_powertrain_torque(double throttle_input, double dt) {
  // Calculate motor RPM from wheel speed and gear ratio
  float avg_wheel_speed = (state_->wheels_speed.rear_left + state_->wheels_speed.rear_right) / 2.0f;
  float wheel_angular_velocity = avg_wheel_speed /
                                 simulator_parameters_->car_parameters->wheel_diameter * 2.0f *
                                 M_PI;  // rad/s
  float motor_omega = wheel_angular_velocity * simulator_parameters_->car_parameters->gear_ratio;
  float motor_rpm = (motor_omega * 60.0f / (2.0f * M_PI));

  // Get maximum torque available at this RPM
  float max_motor_torque = motor_->getMaxTorqueAtRPM(motor_rpm);

  // Step 1: Torque Request and Mechanical Power
  float requested_motor_torque = throttle_input * max_motor_torque;
  float mechanical_power_req = requested_motor_torque * motor_omega;

  // Step 2: Electrical Power Requirement
  float efficiency = motor_->getEfficiency(requested_motor_torque, motor_rpm);
  if (efficiency < 0.01f) {
    efficiency = 0.01f;  // Avoid division by zero
  }

  float electrical_power_req = 0.0f;
  if (mechanical_power_req >= 0.0f) {
    // Motoring
    electrical_power_req = mechanical_power_req / efficiency;
  } else {
    // Regenerative braking
    electrical_power_req = mechanical_power_req * efficiency;
  }

  // Step 3 & 4: Battery Current and Power Calculation with Limits
  // Battery model handles discriminant, all electrical limits (Imax, Vmin), and populates
  // both the feasible current and actual electrical power available
  float actual_current = 0.0f;
  float electrical_power_avail = 0.0f;
  std::tuple<float, float> battery_output =
      battery_->calculateCurrentForPower(electrical_power_req);

  actual_current = std::get<0>(battery_output);
  electrical_power_avail = std::get<1>(battery_output);

  // Step 5: Available Torque Calculation
  float actual_motor_torque = 0.0f;
  float safe_omega = std::max(std::abs(motor_omega), 0.01f);
  if (electrical_power_avail >= 0.0f) {
    actual_motor_torque = (electrical_power_avail * efficiency) / safe_omega;
  } else {
    actual_motor_torque = (electrical_power_avail / efficiency) / safe_omega;
  }
  // Clamp to max torque
  actual_motor_torque = std::clamp(actual_motor_torque, -max_motor_torque, max_motor_torque);

  // Update battery and motor states
  battery_->updateState(actual_current, dt);
  motor_->updateState(actual_current, actual_motor_torque, dt);

  return actual_motor_torque;
}

//double vx, double vy, double steering_angle,double yaw_rate, double dist_to_cg,  bool isLeft
double FSFEUP02Model::calculateSlipAngleFront(double dist_to_cg , bool isLeft){
    const double V_eps = 0.5; // Regularization constant to prevent incorrect low speed behavior can be lower if needed
    double sign = (isLeft) ? -1.0 : 1.0; // Sign used to apply the effect of yaw_rate
    //Normalize velocity to wheels coordinate system
    double Vcx = state_->vx * cos(state_->steering_angle) + state_->vy * sin(state_->steering_angle);
    double Vcy = - state_->vx * sin(state_->steering_angle) + state_->vy * cos(state_->steering_angle);
    return -(atan((Vcy+ (state_->yaw_rate * dist_to_cg) + (sign * state_->yaw_rate * simulator_parameters_->car_parameters->track_width/2))/sqrt(Vcx * Vcx + V_eps * V_eps)));
}

double FSFEUP02Model::calculateSlipAngleRear(double dist_to_cg , bool isLeft){
    const double V_eps = 0.5; // Regularization constant to prevent incorrect low speed behavior can be lower if needed
    double sign = (isLeft) ? -1.0 : 1.0; // Sign used to apply the effect of yaw_rate
    //Normalize velocity to wheels coordinate system
    double Vcy_contact = state_->vy - (state_->yaw_rate * dist_to_cg) + (sign * state_->yaw_rate * (simulator_parameters_->car_parameters->track_width/2)); // velcoity at the contact patch assuming vcy = vy
    return -(atan((Vcy_contact - (state_->yaw_rate * dist_to_cg) + (sign * state_->yaw_rate * simulator_parameters_->car_parameters->track_width/2))/sqrt(state_->vx * state_->vx + V_eps * V_eps)));
}


double FSFEUP02Model::calculateSlipRatio(double wheel_angular_velocity){
    const double V_eps = 0.5; // Regularization constant to prevent incorrect low speed behavior can be lower if needed
    //Normalize velocity to wheels coordinate system
    double Vcx = state_->vx * cos(state_->steering_angle) + state_->vy * sin(state_->steering_angle);
    double slip_ratio = ( wheel_angular_velocity* simulator_parameters_->car_parameters->tire_parameters->effective_tire_r - Vcx) / sqrt(Vcx * Vcx +V_eps * V_eps );
    return slip_ratio;
}

Eigen::Vector3d FSFEUP02Model::calculateTireForces(std::string tire_name, double load) {
  double slip_a = 0.0;
  double slip_r = 0.0;
  TireInput tire_input;
  tire_input.vx = state_->vx;
  tire_input.vy = state_->vy;
  tire_input.yaw_rate = state_->yaw_rate;
  tire_input.steering_angle = state_->steering_angle;
  tire_input.vertical_load = load;
  if (tire_name == "fl") {
    tire_input.slip_angle = calculateSlipAngleFront(simulator_parameters_->car_parameters->tire_parameters->distance_to_CG, true);
    tire_input.slip_ratio = calculateSlipRatio(state_->wheels_speed.front_left); // Assuming wheel speed is in rad/s
    tire_input.wheel_angular_speed = state_->wheels_speed.front_left;
    return front_left->tire_forces(tire_input);
  }
  else if (tire_name == "fr") {
    tire_input.slip_angle = calculateSlipAngleFront(simulator_parameters_->car_parameters->tire_parameters->distance_to_CG, false);
    tire_input.slip_ratio = calculateSlipRatio(state_->wheels_speed.front_right); // Assuming wheel speed is in rad/s
    tire_input.wheel_angular_speed = state_->wheels_speed.front_right;
    return front_right->tire_forces(tire_input);
  }
  else if (tire_name == "rl") {
    tire_input.slip_angle = calculateSlipAngleRear(simulator_parameters_->car_parameters->tire_parameters->distance_to_CG, true);
    tire_input.slip_ratio = calculateSlipRatio(state_->wheels_speed.rear_left); // Assuming wheel speed is in rad/s
    tire_input.wheel_angular_speed = state_->wheels_speed.rear_left;
    return rear_left->tire_forces(tire_input);
  }
  else if (tire_name == "rr") { 
    tire_input.slip_angle = calculateSlipAngleRear(simulator_parameters_->car_parameters->tire_parameters->distance_to_CG, false);
    tire_input.slip_ratio = calculateSlipRatio(state_->wheels_speed.rear_right); // Assuming wheel speed is in rad/s
    tire_input.wheel_angular_speed = state_->wheels_speed.rear_right;
    return rear_right->tire_forces(tire_input);
  }
  else{
      throw std::invalid_argument("Invalid tire name");
  }
}