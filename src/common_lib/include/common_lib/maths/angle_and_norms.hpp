#ifndef SRC_COMMON_LIB_INCLUDE_COMMON_LIB_MATHS_ANGLE_AND_NORMS_HPP_
#define SRC_COMMON_LIB_INCLUDE_COMMON_LIB_MATHS_ANGLE_AND_NORMS_HPP_

#include <cmath>

#include "common_lib/structures/angle_and_norms.hpp"
#include "common_lib/structures/position.hpp"

namespace common_lib::maths {

using TwoDVector = common_lib::structures::Position;
/**
 * @brief Function used to calculate the angle between two vectors and their norms
 *
 * @param vector1 2d vector represented by a pair of doubles
 * @param vector2 2d vector represented by a pair of doubles
 * @return double
 */
common_lib::structures::AngleAndNorms angle_and_norms(const TwoDVector& vector1,
                                                      const TwoDVector& vector2);
}  // namespace common_lib::maths

#endif  // SRC_COMMON_LIB_INCLUDE_COMMON_LIB_MATHS_ANGLE_AND_NORM_HPP_