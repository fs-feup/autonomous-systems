#include "common_lib/maths/angle_and_norms.hpp"

common_lib::structures::AngleAndNorms common_lib::maths::angle_and_norms(
    const common_lib::maths::TwoDVector& vector1, const common_lib::maths::TwoDVector& vector2) {
  double norm1 = std::sqrt(vector1.x * vector1.x + vector1.y * vector1.y);
  double norm2 = std::sqrt(vector2.x * vector2.x + vector2.y * vector2.y);
  double dot_product = vector1.x * vector2.x + vector1.y * vector2.y;
  double angle = std::acos(dot_product / (norm1 * norm2));
  return common_lib::structures::AngleAndNorms(angle, norm1, norm2);
}