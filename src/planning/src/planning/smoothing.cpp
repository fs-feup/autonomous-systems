#include "planning/smoothing.hpp"

void PathSmoothing::order_path(std::vector<PathPoint>& unord_path, const Pose& car_pose) {
  std::unordered_set<PathPoint> unord_set(unord_path.begin(), unord_path.end());

  PathPoint current_point = PathPoint(car_pose.position, 1);
  int index = 0;
  while (!unord_set.empty()) {
    double min_distance = std::numeric_limits<double>::max();
    PathPoint closest_point;

    for (const auto& point : unord_set) {
      if (index == 0){
        float path_angle = atan2(point.position.y - current_point.position.y, point.position.x - current_point.position.x);
        // Check if next point is in the same direction range or side as the vehicle orientation
         if (abs(fmod(path_angle - car_pose.orientation, 2 * M_PI) - M_PI) > M_PI / 2)
            continue;
      }

      double distance = current_point.position.euclidean_distance(point.position);
      if (distance < min_distance) {
        min_distance = distance;
        closest_point = point;
      }
    }

    current_point = closest_point;
    unord_set.erase(current_point);
    unord_path[index++] = current_point;
  }
}

std::vector<PathPoint> PathSmoothing::smooth_path(std::vector<PathPoint>& unordered_path,
                                                  const Pose& car_pose) {
  order_path(unordered_path, car_pose);
  return fit_spline(this->config_.precision, this->config_.order, this->config_.coeffs_ratio,
                    unordered_path);
}