#pragma once

#include <memory>
#include <algorithm>

#include "base_lateral_controller.hpp"
#include "solver/map.hpp"
#include "local_pather/map.hpp"
#include "utils/path_stager.hpp"

class MPCzinho : public LateralController{
private:
  std::shared_ptr<SolverInterface> solver_;
  custom_interfaces::msg::VehicleStateVector solver_state_ = custom_interfaces::msg::VehicleStateVector(); // state vector for the solver
  bool _path_received_ = false;
  custom_interfaces::msg::PathPointArray latest_path_;
  std::shared_ptr<LocalPather> local_pather_;
  void set_path_in_solver();
public:
  /**
   * @brief Construct a new MPCzinho object
   * @param params Control parameters
   */
  MPCzinho(const ControlParameters& params);

  void path_callback(const custom_interfaces::msg::PathPointArray& msg) override;

  void vehicle_state_callback(const custom_interfaces::msg::VehicleStateVector& msg) override;

  void vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) override;

  double get_steering_command() override;

  void publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) override;
};
