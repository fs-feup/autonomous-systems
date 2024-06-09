#include <ros/ros.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <string>
#include <vector>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "gtest/gtest.h"
#include "planning/cone_coloring2.hpp"
#include "planning/outliers.hpp"
#include "planning/smoothing2.hpp"
// #include "planning/local_path_planner.hpp"
// #include "planning/path_smoothing.hpp"
// #include "planning/track.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/cone_coloring_config.hpp"
// #include "utils/files.hpp"
#include "utils/splines.hpp"

using testing::Eq;
namespace fs = std::filesystem;

std::ifstream open_read_file(const std::string &filename) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Start openReadFile");
  std::string filePrefix = ament_index_cpp::get_package_share_directory("planning");
  filePrefix = filePrefix.substr(0, filePrefix.find("install"));
  std::string logger_variable = filePrefix + filename;
  std::ifstream file(filePrefix + filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR opening file: %s\n", logger_variable.c_str());
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Successfully opened %s \n",
                 logger_variable.c_str());
  }
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "End openReadFile");
  return file;
}

std::vector<common_lib::structures::Cone> cone_vector_from_file(const std::string &path) {
  std::string x, y, color;
  std::ifstream trackFile = open_read_file(path);
  std::vector<common_lib::structures::Cone> output;
  while (trackFile >> x >> y >> color) {
    float xValue = stof(x);
    float yValue = stof(y);
    common_lib::competition_logic::Color cone_color;
    if (color == "blue_cone") {
      cone_color = common_lib::competition_logic::Color::BLUE;
    } else if (color == "yellow_cone") {
      cone_color = common_lib::competition_logic::Color::YELLOW;
    } else if (color == "orange_cone") {
      cone_color = common_lib::competition_logic::Color::ORANGE;
    } else if (color == "large_orange_cone") {
      cone_color = common_lib::competition_logic::Color::LARGE_ORANGE;
    } else {
      cone_color = common_lib::competition_logic::Color::UNKNOWN;
    }
    common_lib::structures::Cone cone;
    cone.position.x = static_cast<double>(xValue);
    cone.position.y = static_cast<double>(yValue);
    cone.color = cone_color;
    output.push_back(cone);
  }
  trackFile.close();
  return output;
}

std::pair<std::vector<common_lib::structures::Cone>, std::vector<common_lib::structures::Cone>>
track_from_file(const std::string &path) {
  std::string x, y, color;
  std::ifstream trackFile = open_read_file(path);
  std::vector<common_lib::structures::Cone> left_output;
  std::vector<common_lib::structures::Cone> right_output;
  while (trackFile >> x >> y >> color) {
    float xValue = stof(x);
    float yValue = stof(y);
    common_lib::structures::Cone cone;
    cone.position.x = static_cast<double>(xValue);
    cone.position.y = static_cast<double>(yValue);
    if (color == "blue_cone") {
      cone.color = common_lib::competition_logic::Color::BLUE;
      left_output.push_back(cone);
    } else if (color == "yellow_cone") {
      cone.color = common_lib::competition_logic::Color::YELLOW;
      right_output.push_back(cone);
    }
  }
  trackFile.close();
  return std::make_pair(left_output, right_output);
}

std::vector<common_lib::structures::PathPoint> path_from_file(const std::string &path) {
  std::string x, y, v;
  std::ifstream path_file = open_read_file(path);
  std::vector<common_lib::structures::PathPoint> output;
  while (path_file >> x >> y >> v) {
    float x_coordinate = stof(x);
    float y_coordinate = stof(y);
    float velocity = stof(v);
    common_lib::structures::PathPoint path_point;
    path_point.position.x = static_cast<double>(x_coordinate);
    path_point.position.y = static_cast<double>(y_coordinate);
    path_point.ideal_velocity = static_cast<double>(velocity);
    output.push_back(path_point);
  }
  path_file.close();
  return output;
}

float consecutive_max_distance(const std::vector<common_lib::structures::Cone> &cones) {
  float maxDistance = 0.0;
  for (size_t i = 1; i < cones.size(); i++) {
    float distance = cones[i].position.euclidean_distance(cones[i - 1].position);
    if (distance > maxDistance) {
      maxDistance = distance;
    }
  }
  return maxDistance;
}

// /**
//  * @brief Defines the way in which two pairs of doubles should be
//  * compared when ordering them (lexicographic comparison)
//  *
//  * @param a One of the pairs of doubles to be compared
//  * @param b One of the pairs of doubles to be compared
//  * @return true if a<b
//  * @return false if a>b
//  */
// bool custom_comparator(const std::pair<double, double> &a, const std::pair<double, double> &b) {
//   if (a.first != b.first) {
//     return a.first < b.first;
//   }
//   return a.second < b.second;
// }
//
// void log_cone1(const Cone *c) {
//   std::cout << "X cone: " << c->getX() << " , Y cone: " << c->getY() << " Id cone: " <<
//   c->getId()
//             << std::endl;
// }
//
// void log_cone2(const Cone *c) { std::cout << "(" << c->getX() << "," << c->getY() << "),"; }
//
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
void test_cone_coloring(
    std::pair<std::vector<common_lib::structures::Cone>, std::vector<common_lib::structures::Cone>>
        track,
    int &correctly_placed_blue, int &correctly_placed_yellow, int &incorrectly_placed_blue,
    int &incorrectly_placed_yellow) {
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
//
// /**
//  * @brief orders a vector of pairs to make it easier to compare them
//  *
//  * @param vec vector of pairs to be ordered
//  * @return ordered vector of pairs
//  */
// std::vector<std::pair<double, double>> orderVectorOfPairs(
//     const std::vector<std::pair<double, double>> &vec) {
//   std::vector<std::pair<double, double>> result = vec;
//   std::sort(result.begin(), result.end(), custom_comparator);
//   return result;
// }
//
// /**
//  * @brief Get current date and time as a string.
//  * @return Current date and time as a string in "YYYY-MM-DD-HH:MM" format.
//  */
// std::string get_current_date_time_as_string() {
//   auto now = std::chrono::system_clock::now();
//   std::time_t now_time = std::chrono::system_clock::to_time_t(now);
//   std::tm now_tm;
//   localtime_r(&now_time, &now_tm);
//
//   std::stringstream ss;
//   ss << std::put_time(&now_tm, "%Y-%m-%d-%H:%M");
//   return ss.str();
// }
//
/**
 * @brief rounds float to n decimal places
 *
 * @param num number to be rounded
 * @param decimal_places number of decimal places
 * @return rounded number
 */
float round_n(float num, int decimal_places) {
  num *= pow(10, decimal_places);
  int intermediate = round(num);
  num = intermediate / pow(10, decimal_places);
  return num;
}
//
// /**
//  * @brief Test function for Delaunay Triangulations' efficiency.
//  * Runs 100 times to get average execution time.
//  *
//  * @param filename path to the file that contains the data for testing
//  * @return path after Delaunay Triangulations
//  */
// std::vector<PathPoint> processTriangulations(std::string filename) {
//   Track *track = new Track();
//   track->fillTrack(filename);
//
//   LocalPathPlanner *pathplanner = new LocalPathPlanner();
//   std::vector<PathPoint> path;
//   std::ofstream measuresPath = openWriteFile("performance/exec_time/planning/planning_" +
//                                              get_current_date_time_as_string() + ".csv");
//   double total_time = 0;
//   int no_iters = 100;
//
//   // No_iters repetitions to get average
//   for (int i = 0; i < no_iters; i++) {
//     auto t0 = std::chrono::high_resolution_clock::now();
//
//     std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
//
//     auto t1 = std::chrono::high_resolution_clock::now();
//
//     double elapsed_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
//
//     total_time += elapsed_time_ms;
//
//     for (size_t i = 0; i < pathPointers.size(); i++) path.push_back(*pathPointers[i]);
//   }
//
//   measuresPath << total_time / no_iters << "\n";
//   measuresPath.close();
//
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Average Delaunay Triangulations processed in %f
//   ms.",
//               (total_time / no_iters));
//
//   return path;
// }
//
// /**
//  * @brief Test function for outlier removal's efficiency.
//  * Runs 100 times and calculates average duration.
//  *
//  * @param filename path to the file that contains the data for testing
//  * @param num_outliers Number of outliers
//  */
// void outlierCalculations(std::string filename, int num_outliers = 0) {
//   Track *track = new Track();
//   track->fillTrack(filename);  // fill track with file data
//
//   // Remove outliers no_iters times to get average
//   int no_iters = 100;
//   double total_time = 0;
//
//   std::ofstream measuresPath = openWriteFile(
//       "performance/exec_time/planning/planning_" + get_current_date_time_as_string() + ".csv",
//       "Number of Left Cones,Number of Right Cones,Number of "
//       "Outliers,Outliers Removal Execution "
//       "Time,Triangulations Execution Time");
//
//   for (int i = 0; i < no_iters; i++) {
//     track->reset();
//     track->fillTrack(filename);
//     auto t0 = std::chrono::high_resolution_clock::now();
//     track->validateCones();
//     auto t1 = std::chrono::high_resolution_clock::now();
//     double elapsed_time_iter_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
//     total_time += elapsed_time_iter_ms;
//   }
//
//   measuresPath << track->getLeftConesSize() << "," << track->getRightConesSize() << ","
//                << num_outliers << "," << total_time / no_iters << ",";
//   measuresPath.close();
//
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Outliers removed in average %f ms.",
//               (total_time / no_iters));
// }
//
// std::ostream &operator<<(std::ostream &os, const PathPoint &p) {
//   return os << '(' << p.getX() << ", " << p.getY() << ')';
// }
//
/**
 * @brief simple case in which there are cones only in the left size
 * and no significant outliers
 *
 */
TEST(LocalPathPlanner, outlier_test2) {
  std::string file_path = "src/planning/tracks/outlier_test2.txt";
  Outliers outliers;
  auto track = track_from_file(file_path);
  int n1_left = track.first.size();
  int n1_right = track.second.size();
  EXPECT_EQ(n1_left, 12);
  EXPECT_EQ(n1_right, 0);
  track = outliers.approximate_cones_with_spline(track);
  n1_left = track.first.size();
  n1_right = track.second.size();
  EXPECT_EQ(n1_left, 12);
  EXPECT_EQ(n1_right, 0);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 2.772);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 0);
  int n2_left = track.first.size();
  int n2_right = track.second.size();
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, 0);
  EXPECT_EQ(n2_right, 0);
}

/**
 * @brief simple scenario only with left cones but with 4 significant outliers
 *
 */
TEST(LocalPathPlanner, outliers_test1) {
  std::string file_path = "src/planning/tracks/outlier_test1.txt";
  Outliers outliers;
  auto track = track_from_file(file_path);
  int n1_left = track.first.size();
  int n1_right = track.second.size();
  track = outliers.approximate_cones_with_spline(track);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 31.1480);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 0);
  int n2_left = track.first.size();
  int n2_right = track.second.size();
  EXPECT_EQ(n1_left, 36);
  EXPECT_EQ(n2_left, 36);
  EXPECT_EQ(n1_right, 0);
  EXPECT_EQ(n2_right, 0);
}

/**
 * @brief scenario with large track but few outliers
 *
 */
TEST(LocalPathPlanner, map250_out10) {
  std::string file_path = "src/planning/tracks/map_250_out10.txt";
  Outliers outliers;
  auto track = track_from_file(file_path);
  int n1_left = track.first.size();
  int n1_right = track.second.size();
  track = outliers.approximate_cones_with_spline(track);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 5.5149999);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 4.901);
  int n2_left = track.first.size();
  int n2_right = track.second.size();
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
}

/**
 * @brief scenario with large track and moderate number of outliers
 *
 */
TEST(LocalPathPlanner, map250_out25) {
  std::string file_path = "src/planning/tracks/map_250_out25.txt";
  Outliers outliers;
  auto track = track_from_file(file_path);
  int n1_left = track.first.size();
  int n1_right = track.second.size();
  track = outliers.approximate_cones_with_spline(track);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 5.5100002);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 6.152);
  int n2_left = track.first.size();
  int n2_right = track.second.size();
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
}

/**
 * @brief scenario with large track and abundant outliers
 *
 */
TEST(LocalPathPlanner, map250_out50) {
  std::string file_path = "src/planning/tracks/map_250_out50.txt";
  Outliers outliers;
  auto track = track_from_file(file_path);
  int n1_left = track.first.size();
  int n1_right = track.second.size();
  track = outliers.approximate_cones_with_spline(track);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 5.4310002);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 7.928);
  int n2_left = track.first.size();
  int n2_right = track.second.size();
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
}
/**
 * @brief scenario with large track that barely has outliers (only 2)
 *
 */
TEST(LocalPathPlanner, map250) {
  std::string file_path = "src/planning/tracks/map_250.txt";
  Outliers outliers;
  auto track = track_from_file(file_path);
  int n1_left = track.first.size();
  int n1_right = track.second.size();
  track = outliers.approximate_cones_with_spline(track);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 4.6900001);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 4.5019999);
  int n2_left = track.first.size();
  int n2_right = track.second.size();
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
}

/**
 * @brief simplest scenario with cones forming a line and one outlier
 *
 */
TEST(LocalPathPlanner, distance_next_outliers) {
  std::string file_path = "src/planning/tracks/distance_to_next.txt";
  auto track = track_from_file(file_path);
  int n1_left = track.first.size();
  int n1_right = track.second.size();
  Outliers outliers;
  track = outliers.approximate_cones_with_spline(track);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.first), 3), 0);
  EXPECT_FLOAT_EQ(round_n(consecutive_max_distance(track.second), 3), 1.483);
  int n2_left = track.first.size();
  int n2_right = track.second.size();
  EXPECT_EQ(n1_left, 0);
  EXPECT_EQ(n2_left, 0);
  EXPECT_EQ(n1_right, 15);
  EXPECT_EQ(n2_right, 15);
}
/**
 * @brief simple scenario with few points in the path
 *
 */
TEST(LocalPathPlanner, path_smooth1) {
  std::string file_path = "src/planning/tracks/path_smooth1.txt";
  PathSmoothing path_smoothing;
  Pose car_pose = {0, 0, 0};
  std::vector<common_lib::structures::PathPoint> input_path = path_from_file(file_path);
  std::vector<common_lib::structures::PathPoint> smoothed_path =
      path_smoothing.smooth_path(input_path, car_pose);
  EXPECT_EQ(smoothed_path.size(), 111);
}
/**
 * @brief more complex scenario with cones deviating from path and with
 * significant curvature
 *
 */
TEST(LocalPathPlanner, path_smooth2) {
  std::string file_path = "src/planning/tracks/path_smooth2.txt";
  PathSmoothing path_smoothing;
  Pose car_pose = {0, 0, 0};
  std::vector<common_lib::structures::PathPoint> input_path = path_from_file(file_path);
  std::vector<common_lib::structures::PathPoint> smoothed_path =
      path_smoothing.smooth_path(input_path, car_pose);
  EXPECT_EQ(smoothed_path.size(), 381);
}
//
// struct PathPointHash {
//   std::size_t operator()(const PathPoint &p) const {
//     // Compute individual hash values for two data members and combine them using XOR
//     std::size_t h1 = std::hash<double>()(p.getX());
//     std::size_t h2 = std::hash<double>()(p.getY());
//
//     return h1 ^ h2;
//   }
// };
//
// struct PathPointEqual {
//   bool operator()(const PathPoint &p1, const PathPoint &p2) const {
//     return p1.getX() == p2.getX() && p1.getY() == p2.getY();
//   }
// };
//
// namespace std {
// template <>
// struct hash<PathPoint> {
//   std::size_t operator()(const PathPoint &p) const {
//     // Compute individual hash values for two data members and combine them using XOR
//     std::size_t h1 = std::hash<double>()(p.getX());
//     std::size_t h2 = std::hash<double>()(p.getY());
//
//     return h1 ^ h2;
//   }
// };
// }  // namespace std
//
// /**
//  * @brief simple but realistic scenario with few cones and fwe outliers
//  *
//  */
// TEST(LocalPathPlanner, delauney10) {
//   std::string file_path = "src/planning/tracks/map_10.txt";
//   Track *track = new Track();
//   track->fillTrack(file_path);  // fill track with file data
//   LocalPathPlanner *pathplanner = new LocalPathPlanner();
//   std::unordered_set<PathPoint> path;
//   std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
//   for (size_t i = 0; i < pathPointers.size(); i++) path.insert(*pathPointers[i]);
//   std::unordered_set<PathPoint> expected{
//       PathPoint(1.5, 0.0),  PathPoint(1.75, 1.0),  PathPoint(2.25, 1.4),
//       PathPoint(2.5, 2.4),  PathPoint(3.0, 2.75),  PathPoint(3.5, 3.15),
//       PathPoint(3.75, 3.9), PathPoint(4.25, 4.25), PathPoint(4.5, 5.0)};
//   EXPECT_EQ(path.size(), expected.size());
//   EXPECT_EQ(expected, path);
// }
//
// /**
//  * @brief scenario: Large piece of track with no outliers
//  *
//  */
// TEST(LocalPathPlanner, delauney3_0) {
//   std::string file_path = "src/planning/tracks/map_100.txt";
//   Track *track = new Track();
//   track->fillTrack(file_path);  // fill track with file data
//   LocalPathPlanner *pathplanner = new LocalPathPlanner();
//   std::vector<std::pair<double, double>> path;
//   std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
//   for (size_t i = 0; i < pathPointers.size(); i++) {
//     path.push_back({pathPointers[i]->getX(), pathPointers[i]->getY()});
//   }
//   std::vector<std::pair<double, double>> expected{
//       {9.264488222047865, -14.544556602878384},  {11.098962470793445, -13.973058825226456},
//       {12.952185068629047, -13.453427875255297}, {14.795742958854717, -12.930775270427054},
//       {16.651178604314083, -12.454028390131437}, {16.889186776896732, -45.89949503428116},
//       {16.99671769430034, -49.85814622291364},   {17.059442621876435, -47.84339828855889},
//       {17.487038416676924, -44.12825293739533},  {18.507199338858825, -42.493363162577126},
//       {18.509420243167266, -11.996424568110662}, {19.433682520335342, -40.8702000504269},
//       {20.370374395591107, -11.577823648488781}, {20.66413537971954, -39.51238691751904},
//       {22.12929977842547, -38.35304645082062},   {22.248899890219143, -11.201472219259287},
//       {23.6818831093663, -37.07882531651846},    {24.118678008948176, -10.85627915130934},
//       {25.471744684974972, -36.15352330202951},  {26.023078470269255, -10.577395793663829},
//       {27.02379989782508, -35.18842366134389},   {27.904986014644113, -10.320872468384028},
//       {28.817295568373027, -34.524893133928074}, {29.83254031662907, -10.166824066409102},
//       {30.552399216759277, -33.719567433006056}, {31.729470914798288, -10.017925311841005},
//       {32.354565874505184, -33.197949369805684}, {33.65376421108042, -10.047864842986382},
//       {34.27112812873541, -32.53544296980906},   {35.71305765436516, -10.062568050365462},
//       {36.09378382946615, -32.034768902581455},  {37.633000153041266, -10.302532803732168},
//       {37.91853508602949, -31.52185188970161},   {39.523423685336915, -10.581239549542051},
//       {39.77312236811387, -30.921543573322403},  {41.26418338575908, -11.045350144388983},
//       {41.49648095138352, -30.46226176510593},   {43.090128395784475, -11.637603386407559},
//       {43.34157475000643, -29.696678914651056},  {44.74864655062539, -12.374454525417715},
//       {45.245568819634066, -29.130963975856815}, {46.340542083295006, -13.256954677913221},
//       {46.93280563822984, -28.245579533030934},  {47.93126862938833, -14.367109164381901},
//       {48.66621796774925, -27.30004239052475},   {49.36583611023501, -15.764303165321168},
//       {50.10491001300154, -26.05721953652515},   {50.662655680863125, -17.299249125396475},
//       {51.06575864950613, -24.477844297198516},  {51.31161508859779, -19.01713029173498},
//       {51.780739560158665, -22.715865481146395}, {51.828563433823994, -20.73905250270295}};
//   path = orderVectorOfPairs(path);
//   expected = orderVectorOfPairs(expected);
//
//   EXPECT_EQ(path.size(), expected.size());
//   if (path.size() == expected.size()) {
//     for (size_t i = 0; i < expected.size(); i++) {
//       EXPECT_LE(fabs(path[i].first - expected[i].first), 0.1);
//       EXPECT_LE(fabs(path[i].second - expected[i].second), 0.1);
//     }
//   }
//   // For some reason,  the current implementation has different
//   // outputs when ran different times. All results are coherent with
//   // Delaunay Triangulations but vary minimally. This should be reviewed later
// }
//
// /**
//  * @brief scenario: cones in parallel straight lines that shows that cones
//  * exaclty in a circumference are not counted as being inside a circle
//  *
//  */
// TEST(LocalPathPlanner, map_test1_void) {
//   std::string file_path = "src/planning/tracks/map_test1.txt";
//   Track *track = new Track();
//   track->fillTrack(file_path);  // fill track with file data
//   LocalPathPlanner *pathplanner = new LocalPathPlanner();
//   std::unordered_set<PathPoint> path;
//   std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
//   for (size_t i = 0; i < pathPointers.size(); i++) path.insert(*pathPointers[i]);
//   std::unordered_set<PathPoint> expected{{1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0},
//                                          {6, 0}, {7, 0}, {8, 0}, {0, 0}};
//   EXPECT_EQ(path.size(), expected.size());
//   EXPECT_EQ(expected, path);
// }
//
// /**
//  * @brief scenario: small track with sharp edge
//  *
//  */
// TEST(LocalPathPlanner, map_test2) {
//   std::string file_path = "src/planning/tracks/map_test2.txt";
//   Track *track = new Track();
//   track->fillTrack(file_path);  // fill track with file data
//   LocalPathPlanner *pathplanner = new LocalPathPlanner();
//   std::unordered_set<PathPoint> path;
//   std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
//   for (size_t i = 0; i < pathPointers.size(); i++) {
//     path.insert({pathPointers[i]->getX(), pathPointers[i]->getY()});
//   }
//   std::unordered_set<PathPoint> expected = {
//       {1, 0}, {1, 1},     {1, 2}, {1.5, 3},   {1.75, 3.5}, {1.75, 3.5}, {1.75, 3.5},
//       {2, 4}, {2.5, 4.5}, {3, 5}, {3.5, 4.5}, {4, 4},      {3.5, 2},    {4.5, 3.5}};
//   EXPECT_EQ(path.size(), expected.size());
//   EXPECT_EQ(path, expected);
// }
//
// /**
//  * Extracts the size and number of outliers from a filename.
//  *
//  * @param filename The filename containing size and number of outliers
//  * information.
//  * @param size Output parameter to store the extracted size.
//  * @param n_outliers Output parameter to store the extracted number of outliers.
//  *
//  * The filename should be in the format "map_{size}_{n_outliers}.txt", where:
//  *   - "size" is an integer representing the size.
//  *   - "n_outliers" is an integer representing the number of outliers.
//  *
//  * Example:
//  *   If filename is "map_100_5.txt", size will be set to 100 and n_outliers
//  * to 5.
//  */
// void extractInfo(const std::string_view &filenameView, int &size, int &n_outliers) {
//   std::string filename(filenameView);
//   size_t pos1 = filename.find("_");
//   size_t pos2 = filename.find("_", pos1 + 1);
//   size_t pos3 = filename.find(".", pos2 + 1);
//   std::string size_str = filename.substr(pos1 + 1, pos2 - pos1 - 1);
//   std::string n_outliers_str = filename.substr(pos2 + 1, pos3 - pos2 - 1);
//   std::istringstream(size_str) >> size;
//   std::istringstream(n_outliers_str) >> n_outliers;
// }

// test the placement of initial cones with the car oriented at 0 rad
TEST(ConeColoring, place_first_cones1) {
  std::vector<common_lib::structures::Cone> track =
      cone_vector_from_file("src/planning/tracks/track1.txt");
  std::unordered_set<common_lib::structures::Cone, std::hash<common_lib::structures::Cone>,
                     ConeAproxEqual>
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

// test the placement of initial cones with the car oriented at pi rad
TEST(ConeColoring, place_first_cones2) {
  std::vector<common_lib::structures::Cone> test_cones =
      cone_vector_from_file("src/planning/tracks/track1.txt");
  std::unordered_set<common_lib::structures::Cone, std::hash<common_lib::structures::Cone>,
                     ConeAproxEqual>
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
  std::cout << "Left cones ";
  for (Cone &c : track.first) {
    std::cout << "(" << c.position.x << "," << c.position.y << "),";
  }
  std::cout << std::endl << "Right cones:";
  for (Cone &c : track.second) {
    std::cout << "(" << c.position.x << "," << c.position.y << "),";
  }
  std::cout << std::endl;
  test_cone_coloring(track, c_left, c_right, inc_left, inc_right);
  EXPECT_EQ(c_left, 128);
  EXPECT_EQ(c_right, 140);
  EXPECT_EQ(inc_left, 0);
  EXPECT_EQ(inc_right, 0);
}

TEST(ConeColoring, fullconecoloring2) {
  ConeColoringConfig config;
  ConeColoring cone_coloring(config);
  for (double ae = 30; ae < 30; ae += 1) {
    std::cout << "-------------------------------" << std::endl
              << "New distance exponent : " << ae << std::endl;
    cone_coloring.config_.max_cost = ae;
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

// Test the spline template function
TEST(Splines, spline1) {
  using PathPoint = common_lib::structures::PathPoint;
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
TEST(Splines, spline2) {
  using Cone = common_lib::structures::Cone;
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