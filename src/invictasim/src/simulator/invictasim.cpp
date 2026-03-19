#include "simulator/invictasim.hpp"

InvictaSim::InvictaSim(const InvictaSimParameters& params)
    : params_(params),
      running_(false),
      sim_time_(0.0),
      throttle_({0.0, 0.0, 0.0, 0.0}),
      steering_(0.0) {
  vehicle_model_ = vehicle_models_map.at(params_.vehicle_model.c_str())(params);
  next_loop_time_ = std::chrono::steady_clock::now();
}

void InvictaSim::run() {
  running_ = true;
  while (running_) {
    simulation_step();
  }
}

void InvictaSim::stop() { running_ = false; }

void InvictaSim::simulation_step() {
  auto now = std::chrono::steady_clock::now();
  if (now < next_loop_time_) {
    std::this_thread::sleep_until(next_loop_time_);
  }
  next_loop_time_ = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(params_.timestep / params_.simulation_speed));

  sim_time_ += params_.timestep;

  // Take copy of input so no locks are required during the vehicle model step
  common_lib::structures::Wheels throttle;
  double steering;
  get_input_snapshot(throttle, steering);

  // Use snapshot throughout step without locks
  vehicle_model_->step(params_.timestep, throttle, steering);
}
