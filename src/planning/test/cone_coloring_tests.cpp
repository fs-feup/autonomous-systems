#include "test_utils/utils.hpp"

/**
 * @brief Function to validate cone coloring. Compares each cone's id to its supposed Id
 * (left side id is even, right side id is odd)
 *
 * @param cone_coloring ConeColoring object after cone coloring was tried
 * @param correctly_placed_blue number of blue cones correctly placed
 * @param correctly_placed_yellow number of yellow cones correctly placed
 * @param incorrectly_placed_blue number of blue cones incorrectly placed
 * @param incorrectly_placed_yellow number of yellow cones incorrectly placed
 */
void test_cone_coloring(const std::pair<std::vector<common_lib::structures::Cone>,
                                        std::vector<common_lib::structures::Cone>> &track,
                        int &correctly_placed_blue, int &correctly_placed_yellow,
                        int &incorrectly_placed_blue, int &incorrectly_placed_yellow) {
  correctly_placed_blue = 0;
  correctly_placed_yellow = 0;
  incorrectly_placed_blue = 0;
  incorrectly_placed_yellow = 0;
  for (const Cone c : track.first) {
    if (c.color == common_lib::competition_logic::Color::BLUE) {
      correctly_placed_blue++;
    } else {
      incorrectly_placed_yellow++;
    }
  }
  for (const Cone c : track.second) {
    if (c.color == common_lib::competition_logic::Color::YELLOW) {
      correctly_placed_yellow++;
    } else {
      incorrectly_placed_blue++;
    }
  }
}

/**
 * @brief test the placement of initial cones with the car oriented at 0 rad
 *
 */
TEST(ConeColoring, place_first_cones1) {
  std::vector<common_lib::structures::Cone> track =
      cone_vector_from_file("src/planning/tracks/track1.txt");
  std::unordered_set<common_lib::structures::Cone, std::hash<common_lib::structures::Cone>>
      uncolored_cones(track.begin(), track.end());
  double gain_angle = 1.0;
  double gain_distance = 1.0;
  double gain_ncones = 1.0;
  double exponent_1 = 1.0;
  double exponent_2 = 1.0;
  double cost_max = 5000.0;
  auto config =
      ConeColoringConfig(gain_angle, gain_distance, gain_ncones, exponent_1, exponent_2, cost_max);
  auto cone_coloring = ConeColoring(config);
  auto initial_car_pose = Pose(30.0, 15.0, 0);
  std::vector<common_lib::structures::Cone> blue_cones;
  std::vector<common_lib::structures::Cone> yellow_cones;
  int colored_cones = 0;
  cone_coloring.place_initial_cones(uncolored_cones, blue_cones, yellow_cones, initial_car_pose,
                                    colored_cones);
  EXPECT_DOUBLE_EQ(round_n(blue_cones[0].position.x, 3), round_n(28.98699951, 3));
  EXPECT_DOUBLE_EQ(round_n(blue_cones[0].position.y, 3), round_n(18.340000, 3));
  EXPECT_EQ(blue_cones[1].color, common_lib::competition_logic::Color::BLUE);
  EXPECT_DOUBLE_EQ(round_n(yellow_cones[0].position.x, 3), round_n(27.8945, 3));
  EXPECT_DOUBLE_EQ(round_n(yellow_cones[0].position.y, 3), round_n(13.0521, 3));
  EXPECT_EQ(yellow_cones[1].color, common_lib::competition_logic::Color::YELLOW);
  EXPECT_DOUBLE_EQ(round_n(blue_cones[1].position.x, 3), round_n(30.98699951, 3));
  EXPECT_DOUBLE_EQ(round_n(blue_cones[1].position.y, 3), round_n(18.340000, 3));
  EXPECT_EQ(blue_cones[1].color, common_lib::competition_logic::Color::BLUE);
  EXPECT_DOUBLE_EQ(round_n(yellow_cones[1].position.x, 3), round_n(29.8945, 3));
  EXPECT_DOUBLE_EQ(round_n(yellow_cones[1].position.y, 3), round_n(13.0521, 3));
  EXPECT_EQ(yellow_cones[1].color, common_lib::competition_logic::Color::YELLOW);
}

/**
 * @brief test the placement of initial cones with the car oriented at pi rad
 *
 */
TEST(ConeColoring, place_first_cones2) {
  std::vector<common_lib::structures::Cone> test_cones =
      cone_vector_from_file("src/planning/tracks/track1.txt");
  std::unordered_set<common_lib::structures::Cone, std::hash<common_lib::structures::Cone>>
      uncolored_cones(test_cones.begin(), test_cones.end());
  double gain_angle = 1.0;
  double gain_distance = 1.0;
  double gain_ncones = 1.0;
  double exponent_1 = 1.0;
  double exponent_2 = 1.0;
  double cost_max = 5000.0;
  auto config =
      ConeColoringConfig(gain_angle, gain_distance, gain_ncones, exponent_1, exponent_2, cost_max);
  auto cone_coloring = ConeColoring(config);
  auto initial_car_pose = Pose(30.0, 15.0, 3.1416);
  std::vector<common_lib::structures::Cone> blue_cones;
  std::vector<common_lib::structures::Cone> yellow_cones;
  int colored_cones = 0;
  cone_coloring.place_initial_cones(uncolored_cones, blue_cones, yellow_cones, initial_car_pose,
                                    colored_cones);
  EXPECT_DOUBLE_EQ(round_n(yellow_cones[0].position.x, 3), round_n(32.98699951, 3));
  EXPECT_DOUBLE_EQ(round_n(yellow_cones[0].position.y, 3), round_n(18.340000, 3));
  EXPECT_EQ(yellow_cones[1].color, common_lib::competition_logic::Color::YELLOW);
  EXPECT_DOUBLE_EQ(round_n(blue_cones[0].position.x, 3), round_n(31.8945, 3));
  EXPECT_DOUBLE_EQ(round_n(blue_cones[0].position.y, 3), round_n(13.0521, 3));
  EXPECT_EQ(blue_cones[1].color, common_lib::competition_logic::Color::BLUE);
  EXPECT_DOUBLE_EQ(round_n(yellow_cones[1].position.x, 3), round_n(30.98699951, 3));
  EXPECT_DOUBLE_EQ(round_n(yellow_cones[1].position.y, 3), round_n(18.340000, 3));
  EXPECT_EQ(yellow_cones[1].color, common_lib::competition_logic::Color::YELLOW);
  EXPECT_DOUBLE_EQ(round_n(blue_cones[1].position.x, 3), round_n(29.8945, 3));
  EXPECT_DOUBLE_EQ(round_n(blue_cones[1].position.y, 3), round_n(13.0521, 3));
  EXPECT_EQ(blue_cones[1].color, common_lib::competition_logic::Color::BLUE);
}

TEST(ConeColoring, fullconecoloring1) {
  std::vector<common_lib::structures::Cone> track_cones =
      cone_vector_from_file("src/planning/tracks/track1.txt");
  int c_right;
  int inc_right;
  int c_left;
  int inc_left;
  auto config = ConeColoringConfig();
  auto cone_coloring = ConeColoring(config);
  auto initial_car_pose = Pose(30.0, 15.0, 3.14);
  auto track = cone_coloring.color_cones(track_cones, initial_car_pose);

  test_cone_coloring(track, c_left, c_right, inc_left, inc_right);
  EXPECT_EQ(c_left, 127);
  EXPECT_EQ(c_right, 139);
  EXPECT_EQ(inc_left, 0);
  EXPECT_EQ(inc_right, 0);
}

TEST(ConeColoring, fullconecoloring2) {
  ConeColoringConfig config;
  ConeColoring cone_coloring(config);
  for (int ae = 30; ae < 30; ae += 1) {
    std::cout << "-------------------------------" << std::endl
              << "New max cost: " << ae << std::endl;
    cone_coloring.config_.max_cost_ = static_cast<double>(ae);
    int average_c_right = 0;
    int average_inc_right = 0;
    int average_c_left = 0;
    int average_inc_left = 0;
    for (int i = 1; i < 21; i++) {
      std::string file_name = "gtruths/tracks/eufs/track" + std::to_string(i) + "/converted_track" +
                              std::to_string(i) + ".txt";
      auto track = cone_vector_from_file(file_name);
      int c_right;
      int inc_right;
      int c_left;
      int inc_left;
      auto initial_car_pose = Pose(0.0, 0.0, 0.0);
      auto result = cone_coloring.color_cones(track, initial_car_pose);
      test_cone_coloring(result, c_left, c_right, inc_left, inc_right);
      average_c_left += c_left;
      average_c_right += c_right;
      average_inc_right += inc_right;
      average_inc_left += inc_left;
      // std::cout << "RESULT \t" << i << "\t\t" << c_left << "\t\t" << c_right << "\t\t" <<
      // inc_left
      //           << "\t\t" << inc_right << std::endl;
    }
    std::cout << "AVERAGE: \t\t" << average_c_left / 20.0 << "\t\t" << average_c_right / 20.0
              << "\t\t" << average_inc_left / 20.0 << "\t\t" << average_inc_right / 20.0
              << std::endl;
  }
}