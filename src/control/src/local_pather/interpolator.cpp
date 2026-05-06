#include "local_pather/interpolator.hpp"

void Interpolator::create_local_path(custom_interfaces::msg::PathPointArray& path_msg, const custom_interfaces::msg::VehicleStateVector& vehicle_state) {
  int path_points_local = 1;
  custom_interfaces::msg::PathPoint tenth_point = path_msg.pathpoint_array[path_points_local];

  // Remove first points
  path_msg.pathpoint_array.erase(path_msg.pathpoint_array.begin(), path_msg.pathpoint_array.begin() + path_points_local);

  double x = vehicle_state.x;
  double y = vehicle_state.y;
  double v = vehicle_state.velocity_x;
  double yaw = vehicle_state.orientation;

  double dx = tenth_point.x - x;
  double dy = tenth_point.y - y;
  double distance = std::sqrt(dx * dx + dy * dy);
  double dv = tenth_point.v - v;
  double dyaw = tenth_point.orientation - yaw;

  
  std::vector<custom_interfaces::msg::PathPoint> interpolated_points;
  for (int i = 0; i < path_points_local; ++i) {
    if (distance > 1e-6) {
      double ratio = i / static_cast<double>(path_points_local);
      custom_interfaces::msg::PathPoint new_point;
      new_point.x = x + ratio * dx;
      new_point.y = y + ratio * dy;
      new_point.v = v + ratio * dv;
      new_point.orientation = yaw + ratio * dyaw;
      interpolated_points.push_back(new_point);
    }
  }

  path_msg.pathpoint_array.insert(path_msg.pathpoint_array.begin(), 
                                   interpolated_points.begin(),
                                   interpolated_points.end());
}