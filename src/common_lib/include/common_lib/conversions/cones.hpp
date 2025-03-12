// This file contains conversions of cones to different formats
#pragma once
#include <Eigen/Dense>
#include <vector>

#include "common_lib/structures/cone.hpp"

namespace common_lib::conversions {
/**
 * @brief Convert vector of the struct common_lib::structures::Cone to Eigen vectors
 *
 * @param cones input cones
 * @param positions output vector of the positions of cones in the format [x1, y1, x2, y2, ...]
 * @param confidences confidence associated with each cone
 */
void cone_vector_to_eigen(const std::vector<common_lib::structures::Cone>& cones,
                          Eigen::VectorXd& positions, Eigen::VectorXd& confidences);

}  // namespace common_lib::conversions