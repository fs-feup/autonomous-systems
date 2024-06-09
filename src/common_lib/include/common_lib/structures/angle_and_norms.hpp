#ifndef SRC_COMMON_LIB_INCLUDE_COMMON_LIB_STRUCTURES_ANGLE_AND_NORMS_HPP_
#define SRC_COMMON_LIB_INCLUDE_COMMON_LIB_STRUCTURES_ANGLE_AND_NORMS_HPP_

/**
 * @brief structure to store the angle formed between two vectors and their norms
 *
 */
namespace common_lib::structures {
struct AngleAndNorms {
  double angle;
  double norm1;
  double norm2;
  AngleAndNorms() = default;
  AngleAndNorms(double angle, double norm1, double norm2)
      : angle(angle), norm1(norm1), norm2(norm2){};
};
}  // namespace common_lib::structures

#endif  // SRC_COMMON_LIB_INCLUDE_COMMON_LIB_STRUCTURES_ANGLE_AND_NORMS_HPP_