#include "node/invictasim_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "loader/config.hpp"
#include "vehicle_model/Bicycle/bicycle.hpp"

InvictaSimNode::InvictaSimNode(const InvictaSimParameters& params)
    : Node("invictasim_node"), params_(params), sim_time_(0.0) {
  // Create vehicle model based on config
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("invictasim");
  std::string vehicle_model_config_path =
      package_share_dir + "/config/vehicle_models/" + params_.vehicle_model_config;
  if (params_.vehicle_model == "bicycle_model") {
    vehicle_model_ = std::make_shared<BicycleModel>(vehicle_model_config_path);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown vehicle model: %s", params_.vehicle_model.c_str());
    throw std::runtime_error("Unknown vehicle model");
  }

  // Publishers, subscribers, etc.  initialized here
  test_pub_ = this->create_publisher<std_msgs::msg::Float64>("/invictasim/test_topic", 10);

  init();
}

void InvictaSimNode::init() {
  // Create simulation timer
  auto timer_period = std::chrono::microseconds(
      static_cast<int>((params_.timestep / params_.simulation_speed) * 1000000.0));
  simulation_timer_ =
      this->create_wall_timer(timer_period, std::bind(&InvictaSimNode::simulation_step, this));

  next_loop_time_ = std::chrono::steady_clock::now();
}

void InvictaSimNode::simulation_step() {
  // Synchronize to real-time: wait until it's time for the next step
  auto now = std::chrono::steady_clock::now();
  if (now < next_loop_time_) {
    std::this_thread::sleep_until(next_loop_time_);
  }
  next_loop_time_ = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(params_.timestep / params_.simulation_speed));

  // Advance simulation time
  sim_time_ += params_.timestep;

  // Update vehicle dynamics
  vehicle_model_->step(params_.timestep);

  // Publish test message
  auto msg = std_msgs::msg::Float64();
  msg.data = sim_time_;
  test_pub_->publish(msg);
}
