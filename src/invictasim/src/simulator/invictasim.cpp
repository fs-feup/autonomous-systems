#include "simulator/invictasim.hpp"

InvictaSim::InvictaSim(const InvictaSimParameters& params,
                       const std::shared_ptr<InvictaSimInputAdapter>& input_adapter,
                       const std::shared_ptr<InvictaSimOutputAdapter>& output_adapter)
    : params_(params),
      input_adapter_(input_adapter),
      output_adapter_(output_adapter),
      running_(false),
      sim_time_(0.0) {
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

  InvictaSimInput input = input_adapter_->get_current_input();
  vehicle_model_->step(params_.timestep, input.throttle, input.steering);

  output_adapter_->publish_outputs(vehicle_model_, input.steering);
}
