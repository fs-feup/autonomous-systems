#include "test_utils/utils.hpp"

/**
 * @brief simple scenario with few points in the path
 *
 */
TEST(PathSmoothing, path_smooth1) {
  std::string file_path = "src/planning/tracks/path_smooth1.txt";
  PathSmoothing path_smoothing;
  Pose car_pose = {0, 0, 0};
  std::vector<common_lib::structures::PathPoint> input_path = path_from_file(file_path);
  std::vector<common_lib::structures::PathPoint> smoothed_path =
      path_smoothing.smooth_path(input_path, car_pose, 0);
  EXPECT_EQ((int)smoothed_path.size(), 120);
}
/**
 * @brief more complex scenario with cones deviating from path and with
 * significant curvature
 *
 */
TEST(PathSmoothing, path_smooth2) {
  std::string file_path = "src/planning/tracks/path_smooth2.txt";
  PathSmoothing path_smoothing;
  Pose car_pose = {0, 0, 0};
  std::vector<common_lib::structures::PathPoint> input_path = path_from_file(file_path);
  std::vector<common_lib::structures::PathPoint> smoothed_path =
      path_smoothing.smooth_path(input_path, car_pose, 0);
  EXPECT_EQ((int)smoothed_path.size(), 390);
}