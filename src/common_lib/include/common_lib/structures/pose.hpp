#pragma once

#include "common_lib/structures/position.hpp"

namespace common_lib::structures {
/**
 * @brief Struct for pose representation
 *
 * @param position Vehicle coordinates, x and y
 * @param orientation Orientation of the vehicle in radians
 * 0 radians is pointing in the positive x direction
 */
struct Pose {
  Position position;
  double orientation = 0.0;                  /// theta in radians
  double orientation_noise = 0.0;            //< theta noise
  rclcpp::Time timestamp = rclcpp::Time(0);  //< Last time the pose was updated
  Pose() = default;
  Pose(Position position, double orientation, double orientation_noise = 0.0,
       rclcpp::Time timestamp = rclcpp::Time(0));
  Pose(double x, double y, double theta, double x_noise = 0.0, double y_noise = 0.0,
       double theta_noise = 0.0, rclcpp::Time timestamp = rclcpp::Time(0));
};

struct VehiclePose : public Pose {
public:
  Position rear_axis_;
  double velocity_;

  // Default constructor
  VehiclePose() = default;

  // Constructor
  VehiclePose(Position cg, Position rear_axis, double orientation, double velocity)
      : Pose(cg, orientation), rear_axis_(rear_axis), velocity_(velocity){};
};
}  // namespace common_lib::structures
