#include "common_lib/maths/angle_and_norms.hpp"

#include <iostream>
common_lib::maths::AngleAndNorms common_lib::maths::angle_and_norms(
    const common_lib::maths::TwoDVector& vector1, const common_lib::maths::TwoDVector& vector2) {
  double norm1 = std::sqrt(vector1.x * vector1.x + vector1.y * vector1.y);
  double norm2 = std::sqrt(vector2.x * vector2.x + vector2.y * vector2.y);
  double dot_product = vector1.x * vector2.x + vector1.y * vector2.y;
  double cossine = dot_product / (norm1 * norm2);
  double angle;
  if (std::fabs(cossine - 1.0) < 1e-3) {
    angle = 0.0;
  } else if (std::fabs(cossine + 1.0) < 1e-3) {
    angle = -M_PI;
  } else {
    angle = std::acos(cossine);
  }
  return common_lib::maths::AngleAndNorms(angle, norm1, norm2);
}

common_lib::maths::AngleAndNorms::AngleAndNorms(const common_lib::maths::TwoDVector& v1,
                                                const common_lib::maths::TwoDVector& v2) {
  *this = common_lib::maths::angle_and_norms(v1, v2);
}