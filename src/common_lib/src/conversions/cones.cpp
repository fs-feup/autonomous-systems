#include "common_lib/conversions/cones.hpp"

void common_lib::conversions::cone_vector_to_eigen(
    const std::vector<common_lib::structures::Cone>& cones, Eigen::SparseMatrix<double>& positions,
    Eigen::SparseMatrix<double>& confidences) {
  if (positions.cols() != 1 || confidences.cols() != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("common_lib"), "Output vectors must have one column");
    return;
  }
  if (2 * static_cast<int>(cones.size()) != positions.size() ||
      static_cast<int>(cones.size()) != confidences.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("common_lib"), "Sizes of input vectors do not match");
    return;
  }

  for (int i = 0; i < static_cast<int>(cones.size()); ++i) {
    positions.coeffRef(2 * i, 0) = cones[i].position.x;
    positions.coeffRef(2 * i + 1, 0) = cones[i].position.y;
    confidences.coeffRef(i, 0) = cones[i].certainty;
  }
}