#include "planning/outliers.hpp"

std::pair<std::vector<Cone>, std::vector<Cone>> Outliers::approximate_cones_with_spline(
    const std::pair<std::vector<Cone>, std::vector<Cone>>& cones) const {
  std::vector<Cone> approximated_left_cones(fit_spline(
      this->config_.precision_, this->config_.order_, this->config_.coeffs_ratio_, cones.first));
  std::vector<Cone> approximated_right_cones(fit_spline(
      this->config_.precision_, this->config_.order_, this->config_.coeffs_ratio_, cones.second));
  for (auto& cone : approximated_left_cones) cone.color = Color::BLUE;
  for (auto& cone : approximated_right_cones) cone.color = Color::YELLOW;
  return std::make_pair(approximated_left_cones, approximated_right_cones);
}