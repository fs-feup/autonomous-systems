#include "adapter_slam/invictasim.hpp"

InvictaSimAdapter::InvictaSimAdapter(const SLAMParameters& params) : SLAMNode(params) {
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = this->_callback_group_;

  this->_operational_status_subscription_ =
      this->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/invictasim/operational_status", 10,
          [this](const custom_interfaces::msg::OperationalStatus::SharedPtr msg) {
            {
              std::unique_lock lock(this->mutex_);
              _go_ = msg->go_signal;
            }
            _mission_ = common_lib::competition_logic::Mission(msg->as_mission);
            this->_slam_solver_->set_mission(_mission_);
          },
          subscription_options);

  if (params.use_simulated_perception_) {
    if (params.slam_optimization_mode_ != "sync") {
      rclcpp::SubscriptionOptions parallel_opts;
      parallel_opts.callback_group = _parallel_callback_group_;
      this->_perception_subscription_ =
          this->create_subscription<custom_interfaces::msg::PerceptionOutput>(
              "/invictasim/perception/cones", 1,
              std::bind(&InvictaSimAdapter::_perception_subscription_callback, this, std::placeholders::_1),
              parallel_opts);
    } else {
      this->_perception_subscription_ =
          this->create_subscription<custom_interfaces::msg::PerceptionOutput>(
              "/invictasim/perception/cones", 1,
              std::bind(&InvictaSimAdapter::_perception_subscription_callback, this, std::placeholders::_1));
    }
  }

  if (params.use_simulated_velocities_) {
    this->_velocities_subscription_ =
        this->create_subscription<custom_interfaces::msg::Velocities>(
            "/invictasim/state_estimation/velocities", 1,
            std::bind(&InvictaSimAdapter::_velocities_subscription_callback, this, std::placeholders::_1),
            subscription_options);
  }
  
  RCLCPP_INFO(this->get_logger(), "InvictaSim SLAM adapter created");
}

void InvictaSimAdapter::finish() {
  RCLCPP_INFO(this->get_logger(), "SLAM finished...");
}
