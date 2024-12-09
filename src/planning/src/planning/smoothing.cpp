#include "planning/smoothing.hpp"

void PathSmoothing::order_path(std::vector<PathPoint>& unord_path, const Pose& car_pose,
                               const double initial_car_orientation) const {
  std::unordered_set<PathPoint> unord_set(unord_path.begin(), unord_path.end());

  PathPoint current_point;

  double car_orientation;
  if (this->config_.use_memory_) {
    current_point = PathPoint(0, 0, 1);
    car_orientation = initial_car_orientation;
  } else {
    car_orientation = car_pose.orientation;

    current_point = PathPoint(car_pose.position.x, car_pose.position.y, 1);
  }
  int index = 0;

  while (!unord_set.empty()) {
    double min_distance = std::numeric_limits<double>::max();
    PathPoint closest_point;

    for (const auto& point : unord_set) {
      // If the algorithm is trying to decide which point is the second in the path, it should
      // guarantee that the orientation that the point gives to the path is aligned with the  car's
      // orientation.
      //  This is not done in the first point because the first point often occurs backwards
      //  relative
      // to the car's pose.
      if (index == 1) {
        double path_angle = atan2(point.position.y - current_point.position.y,
                                  point.position.x - current_point.position.x);

        // Check if next point is in the same direction range or side as the vehicle orientation
        double angle_diff = fmod(path_angle - car_orientation + 2 * M_PI, 2 * M_PI);
        if (abs(angle_diff) > M_PI / 2 && abs(angle_diff) < 3 * M_PI / 2) {
          continue;
        }
      }

      double distance = current_point.position.euclidean_distance(point.position);
      if (distance < min_distance) {
        min_distance = distance;
        closest_point = point;
      }
    }

    if ((min_distance < MAX_DISTANCE_BETWEEN_POINTS || index == 0) &&
        index < static_cast<int>(unord_path.size())) {
      current_point = closest_point;
      unord_set.erase(current_point);
      unord_path[index] = current_point;
      index++;
    } else {
      RCLCPP_DEBUG(
          rclcpp::get_logger("planning"),
          "Index out of bounds while ordering path OR no valid point found. index: %d, min "
          "distance: %f",
          index, min_distance);
      break;
    }
  }
  unord_path.erase(unord_path.begin() + index, unord_path.end());
}

std::vector<PathPoint> PathSmoothing::smooth_path(std::vector<PathPoint>& unordered_path,
                                                  const Pose& car_pose,
                                                  const double initial_car_orientation) const {
  order_path(unordered_path, car_pose, initial_car_orientation);
  if (this->config_.use_path_smoothing_) {
    return fit_spline(this->config_.precision_, this->config_.order_, this->config_.coeffs_ratio_,
                      unordered_path);
  } else {
    return unordered_path;
  }
}