#include "test_utils/utils.hpp"

std::vector<common_lib::structures::PathPoint> setUp(std::string file_path){
  std::string adapter;
  PlanningParameters params = Planning::load_config(adapter);
  PlanningConfig planning_config = PlanningConfig(params);
  VelocityPlanning velocity_planning= VelocityPlanning(planning_config.velocity_planning_);

  std::vector<common_lib::structures::PathPoint> final_path = path_from_file(file_path);
  velocity_planning.set_velocity(final_path);
  return final_path;
}
/**
 * @brief simple scenario with few points in the path
 *
 */
TEST(VelocityPlanning, velocity0) {
  std::string file_path = "src/planning/tracks/velocity1.txt";
  std::vector<common_lib::structures::PathPoint> final_path = setUp(file_path);

  EXPECT_EQ((int)final_path.size(), 14);
  EXPECT_EQ(final_path.back().ideal_velocity, 3);
}

/**
 * @brief Check if the velocity is always greater than the minimum velocity
 *
 */
TEST(VelocityPlanning, velocity1) {
  std::string file_path = "src/planning/tracks/velocity1.txt";
  std::vector<common_lib::structures::PathPoint> final_path = setUp(file_path);

  for (std::size_t i = 0; i < final_path.size(); i++) {
    EXPECT_TRUE(final_path[i].ideal_velocity >= 3);
  }
}

/**
 * @brief test for a path with only one point
 *
 */
TEST(VelocityPlanning, velocity2) {
  std::string file_path = "src/planning/tracks/velocity2.txt";
  std::vector<common_lib::structures::PathPoint> final_path = setUp(file_path);

  EXPECT_EQ((int)final_path.size(), 1);
  EXPECT_EQ(final_path[0].ideal_velocity, 3);
}


/**
 * @brief test for a path with no points
 *
 */
TEST(VelocityPlanning, velocity3) {
  std::string file_path = "src/planning/tracks/velocity3.txt";
  std::vector<common_lib::structures::PathPoint> final_path = setUp(file_path);

  EXPECT_EQ((int)final_path.size(), 0);
}