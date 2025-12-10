#include "test_utils/utils.hpp"

/**
 * @brief Test the spline template function for a path point vector
 *
 */
TEST(Splines, spline1) {
  std::vector<PathPoint> cones;
  for (int i = 0; i < 10; i++) {
    PathPoint c;
    c.position.x = i;
    c.position.y = i;
    cones.push_back(c);
  }
  std::vector<PathPoint> vector2 = fit_spline(1, 3, 3.0, cones);
  for (int i = 0; i < 10; i++) {
    EXPECT_LE(fabs(vector2[i].position.x - i), 0.1);
    EXPECT_LE(fabs(vector2[i].position.y - i), 0.1);
  }
}

/**
 * @brief Test the spline template function for a cone vector
 *
 */
TEST(Splines, spline2) {
  std::vector<Cone> cones;
  for (int i = 0; i < 10; i++) {
    Cone c;
    c.position.x = i;
    c.position.y = i;
    cones.push_back(c);
  }
  std::vector<Cone> vector2 = fit_spline(1, 3, 3.0, cones);
  for (int i = 0; i < 10; i++) {
    EXPECT_LE(fabs(vector2[i].position.x - i), 0.1);
    EXPECT_LE(fabs(vector2[i].position.y - i), 0.1);
  }
}