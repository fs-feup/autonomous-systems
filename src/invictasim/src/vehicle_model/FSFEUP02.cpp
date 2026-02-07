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
}

void FSFEUP02Model::step(double dt, double angle, double throttle) {
  float just_to_compile = dt * angle * throttle;  // TODO: implement the actual step function, this
                                                  // is just to make it compile for now
  (void)just_to_compile;                          // to avoid unused variable warning
}

void FSFEUP02Model::reset() {
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;
  vx_ = 0.0;
}

void FSFEUP02Model::set_position(double x, double y, double yaw) {
  x_ = x;
  y_ = y;
  yaw_ = yaw;
}

void FSFEUP02Model::set_velocity(double vx) { vx_ = vx; }

double FSFEUP02Model::get_position_x() const { return x_; }

double FSFEUP02Model::get_position_y() const { return y_; }

double FSFEUP02Model::get_yaw() const { return yaw_; }

double FSFEUP02Model::get_velocity_x() const { return vx_; }

std::string FSFEUP02Model::get_model_name() const { return "FSFEUP02Model"; }
