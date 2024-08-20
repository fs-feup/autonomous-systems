#ifndef SRC_COMMON_LIB_INCLUDE_COMMON_LIB_MATHS_ANGLE_AND_NORMS_HPP_
#define SRC_COMMON_LIB_INCLUDE_COMMON_LIB_MATHS_ANGLE_AND_NORMS_HPP_

#include <cmath>

#include "common_lib/structures/position.hpp"

namespace common_lib::maths {

using TwoDVector = common_lib::structures::Position;

/**
 * @brief structure to store the angle formed between two vectors and their norms
 *
 */
struct AngleAndNorms {
  double angle_;
  double norm1_;
  double norm2_;
  AngleAndNorms() = default;
  AngleAndNorms(double angle, double norm1, double norm2)
      : angle_(angle), norm1_(norm1), norm2_(norm2){};
  AngleAndNorms(const TwoDVector& v1, const TwoDVector& v2);
};

/**
 * @brief Function used to calculate the angle between two vectors and their norms
 *
 * @param vector1 2d vector represented by a pair of doubles
 * @param vector2 2d vector represented by a pair of doubles
 * @return double
 */
AngleAndNorms angle_and_norms(const TwoDVector& vector1, const TwoDVector& vector2);
}  // namespace common_lib::maths

#endif  // SRC_COMMON_LIB_INCLUDE_COMMON_LIB_MATHS_ANGLE_AND_NORM_HPP_