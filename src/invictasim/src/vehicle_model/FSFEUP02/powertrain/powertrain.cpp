#include "vehicle_model/FSFEUP02/powertrain/powertrain.hpp"

PowertrainModel::PowertrainModel(const PowertrainParameters& params) : params_(params) {
  motor_ = std::make_shared<MotorModel>(params_.motor);
  battery_ = std::make_shared<BatteryModel>(params_.battery);
  // differential_ missing
}

Wheels PowertrainModel::calculateWheelTorque(float dt, float throttle_input, Wheels wheel_speed) {
  throttle_input = std::clamp(throttle_input, -1.0f, 1.0f);

  // Calculate motor RPM from wheel speed and gear ratio
  float avg_wheel_speed = (wheel_speed.rear_left + wheel_speed.rear_right) / 2.0f;
  float wheel_angular_velocity = avg_wheel_speed / params_.wheel_radius;
  float motor_omega = wheel_angular_velocity * params_.gear_ratio;
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
  battery_->calculateCurrentForPower(electrical_power_req, actual_current, electrical_power_avail);

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

  // Convert motor torque to wheel torque, WILL BE DONE IN A DIFFERENTIAL MODEL LATER
  float wheel_torque_total = actual_motor_torque * params_.gear_ratio;
  Wheels torque = {0.0f, 0.0f, wheel_torque_total / 2.0f, wheel_torque_total / 2.0f};

  // Update battery and motor states
  battery_->updateState(actual_current, dt);
  motor_->updateState(actual_current, dt);

  return torque;
}

float PowertrainModel::getBatterySOC() const { return battery_->getStateOfCharge(); }

float PowertrainModel::getEnergyConsumed() const { return battery_->getEnergyConsumed(); }

void PowertrainModel::reset() {
  battery_->reset();
  motor_->reset();
}
