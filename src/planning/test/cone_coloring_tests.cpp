#include "test_utils/utils.hpp"

/**
 * @brief Function to validate cone coloring. Compares each cone's id to its supposed Id
 * (left side id is even, right side id is odd)
 *
 * @param cone_coloring ConeColoring object after cone coloring was tried
 * @param correctly_placed_blue number of blue cones correctly placed
 * @param correctly_placed_yellow number of yellow cones correctly placed
 * @param incorrectly_placed_blue number of blue cones incorrectly marked as yellow
 * @param incorrectly_placed_yellow number of yellow cones incorrectly marked as blue
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
 * @brief test the placement of initial cones with the car oriented at 0 rad. The cones' colours
 * should be wrong because the track is meant to be driven in the opposite way.
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
  int colored_cones = 0;
  cone_coloring.place_initial_cones(uncolored_cones, initial_car_pose, colored_cones);
  EXPECT_NEAR(cone_coloring.colored_blue_cones_[0].position.x, 30.986999, 0.001);
  EXPECT_NEAR(cone_coloring.colored_blue_cones_[0].position.y, 18.340000, 0.001);
  EXPECT_EQ(cone_coloring.colored_blue_cones_[1].color,
            common_lib::competition_logic::Color::YELLOW);
  EXPECT_NEAR(cone_coloring.colored_yellow_cones_[0].position.x, 29.8945, 0.001);
  EXPECT_NEAR(cone_coloring.colored_yellow_cones_[0].position.y, 13.0521, 0.001);
  EXPECT_EQ(cone_coloring.colored_yellow_cones_[1].color,
            common_lib::competition_logic::Color::BLUE);
  EXPECT_NEAR(cone_coloring.colored_blue_cones_[1].position.x, 34.640998, 0.001);
  EXPECT_NEAR(cone_coloring.colored_blue_cones_[1].position.y, 17.208000, 0.001);
  EXPECT_EQ(cone_coloring.colored_blue_cones_[1].color,
            common_lib::competition_logic::Color::YELLOW);
  EXPECT_NEAR(cone_coloring.colored_yellow_cones_[1].position.x, 33.45899, 0.001);
  EXPECT_NEAR(cone_coloring.colored_yellow_cones_[1].position.y, 12.34300, 0.001);
  EXPECT_EQ(cone_coloring.colored_yellow_cones_[1].color,
            common_lib::competition_logic::Color::BLUE);
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
  int colored_cones = 0;
  cone_coloring.place_initial_cones(uncolored_cones, initial_car_pose, colored_cones);
  EXPECT_NEAR(cone_coloring.colored_yellow_cones_[0].position.x, 30.986999, 0.001);
  EXPECT_NEAR(cone_coloring.colored_yellow_cones_[0].position.y, 18.340000, 0.001);
  EXPECT_EQ(cone_coloring.colored_yellow_cones_[1].color,
            common_lib::competition_logic::Color::YELLOW);
  EXPECT_NEAR(cone_coloring.colored_blue_cones_[0].position.x, 29.8950, 0.001);
  EXPECT_NEAR(cone_coloring.colored_blue_cones_[0].position.y, 13.0521, 0.001);
  EXPECT_EQ(cone_coloring.colored_blue_cones_[1].color, common_lib::competition_logic::Color::BLUE);
  EXPECT_NEAR(cone_coloring.colored_yellow_cones_[1].position.x, 27.408000, 0.001);
  EXPECT_NEAR(cone_coloring.colored_yellow_cones_[1].position.y, 17.923999, 0.001);
  EXPECT_EQ(cone_coloring.colored_yellow_cones_[1].color,
            common_lib::competition_logic::Color::YELLOW);
  EXPECT_NEAR(cone_coloring.colored_blue_cones_[1].position.x, 26.46400, 0.001);
  EXPECT_NEAR(cone_coloring.colored_blue_cones_[1].position.y, 11.49600, 0.001);
  EXPECT_EQ(cone_coloring.colored_blue_cones_[1].color, common_lib::competition_logic::Color::BLUE);
}

/**
 * @brief test the full cone coloring algorithm in a large track
 *
 */
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
  EXPECT_EQ(c_left, 11);
  EXPECT_EQ(c_right, 12);
  EXPECT_EQ(inc_left, 0);
  EXPECT_EQ(inc_right, 1);
}

/**
 * @brief test the full cone coloring algorithm in 20 tracks. Export the results to a .csv file.
 *
 */
TEST(ConeColoring, fullconecoloring2) {
  ConeColoringConfig config;
  ConeColoring cone_coloring(config);
  int sum_correct_right = 0;    // total actual yellow cones marked as yellow by the algorithm
  int sum_incorrect_right = 0;  // total yellow cones incorrectly marked as blue by the algorithm
  int sum_correct_left = 0;     // total actual blue cones marked as blue by the algorithm
  int sum_incorrect_left = 0;   // total blue cones incorrectly marked as yellow by the algorithm
  std::ofstream
      csv_output_file =  // .csv file to store the results of running the algorithm in 20 tracks
      openWriteFile("performance/other_metrics/planning/cone_coloring_results_" +
                        get_current_date_time_as_string() + ".csv",
                    "Track name, Correctly coloured blue cones, Correctly coloured yellow cones, "
                    "Incorrectly coloured blue cones, Incorrectly coloured yellow cones");
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
    sum_correct_left += c_left;
    sum_correct_right += c_right;
    sum_incorrect_right += inc_right;
    sum_incorrect_left += inc_left;
    csv_output_file << "eufs/track" << i << "," << c_left << "," << c_right << "," << inc_left
                    << "," << inc_right << std::endl;
  }
  double average_correct_blue = static_cast<double>(sum_correct_left) / 20.0;
  double average_correct_yellow = static_cast<double>(sum_correct_right) / 20.0;
  double average_incorrect_blue = static_cast<double>(sum_incorrect_left) / 20.0;
  double average_incorrect_yellow = static_cast<double>(sum_incorrect_right) / 20.0;
  csv_output_file << "Average," << average_correct_blue << "," << average_correct_yellow << ","
                  << average_incorrect_blue << "," << average_incorrect_yellow << std::endl;
}