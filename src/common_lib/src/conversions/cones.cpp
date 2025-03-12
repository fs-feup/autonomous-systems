#include "common_lib/conversions/cones.hpp"

void common_lib::conversions::cone_vector_to_eigen(
    const std::vector<common_lib::structures::Cone>& cones, Eigen::VectorXd& positions,
    Eigen::VectorXd& confidences) {
  if (2 * static_cast<int>(cones.size()) != positions.size() ||
      static_cast<int>(cones.size()) != confidences.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("common_lib"), "Sizes of input vectors do not match");
    return;
  }

  for (int i = 0; i < static_cast<int>(cones.size()); ++i) {
    positions(2 * i) = cones[i].position.x;
    positions(2 * i + 1) = cones[i].position.y;
    confidences(i) = cones[i].certainty;
  }
}