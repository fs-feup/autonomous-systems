#include <ros/ros.h>

#include <chrono>
#include <fstream>
#include <regex>
#include <set>

#include "gtest/gtest.h"
#include "planning/cone_coloring.hpp"
#include "planning/global_path_planner.hpp"
#include "planning/local_path_planner.hpp"
#include "planning/path_smoothing.hpp"
#include "planning/track.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/files.hpp"

using testing::Eq;
namespace fs = std::filesystem;

/**
 * @brief Defines the way in which two pairs of doubles should be
 * compared when ordering them (lexicographic comparison)
 *
 * @param a One of the pairs of doubles to be compared
 * @param b One of the pairs of doubles to be compared
 * @return true if a<b
 * @return false if a>b
 */
bool custom_comparator(const std::pair<double, double> &a, const std::pair<double, double> &b) {
  if (a.first != b.first) {
    return a.first < b.first;
  }
  return a.second < b.second;
}

void log_cone1(const Cone *c) {
  std::cout << "X cone: " << c->getX() << " , Y cone: " << c->getY() << " Id cone: " << c->getId()
            << std::endl;
}

void log_cone2(const Cone *c) { std::cout << "(" << c->getX() << "," << c->getY() << "),"; }

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
void test_cone_coloring(const ConeColoring &cone_coloring, int &correctly_placed_blue,
                        int &correctly_placed_yellow, int &incorrectly_placed_blue,
                        int &incorrectly_placed_yellow) {
  correctly_placed_blue = 0;
  correctly_placed_yellow = 0;
  incorrectly_placed_blue = 0;
  incorrectly_placed_yellow = 0;
  for (const Cone *c : cone_coloring.current_left_cones) {
    if ((c->getId()) % 2) {
      incorrectly_placed_yellow++;
    } else {
      correctly_placed_blue++;
    }
  }
  for (const Cone *c : cone_coloring.current_right_cones) {
    if ((c->getId()) % 2) {
      correctly_placed_yellow++;
    } else {
      incorrectly_placed_blue++;
    }
  }
}

/**
 * @brief orders a vector of pairs to make it easier to compare them
 *
 * @param vec vector of pairs to be ordered
 * @return ordered vector of pairs
 */
std::vector<std::pair<double, double>> orderVectorOfPairs(
    const std::vector<std::pair<double, double>> &vec) {
  std::vector<std::pair<double, double>> result = vec;
  std::sort(result.begin(), result.end(), custom_comparator);
  return result;
}

/**
 * @brief Get current date and time as a string.
 * @return Current date and time as a string in "YYYY-MM-DD-HH:MM" format.
 */
std::string get_current_date_time_as_string() {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm;
  localtime_r(&now_time, &now_tm);

  std::stringstream ss;
  ss << std::put_time(&now_tm, "%Y-%m-%d-%H:%M");
  return ss.str();
}

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

/**
 * @brief Test function for Delaunay Triangulations' efficiency.
 * Runs 100 times to get average execution time.
 *
 * @param filename path to the file that contains the data for testing
 * @return path after Delaunay Triangulations
 */
std::vector<PathPoint> processTriangulations(std::string filename) {
  Track *track = new Track();
  track->fillTrack(filename);

  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::vector<PathPoint> path;
  std::ofstream measuresPath = openWriteFile("performance/exec_time/planning/planning_" +
                                             get_current_date_time_as_string() + ".csv");
  double total_time = 0;
  int no_iters = 100;

  // No_iters repetitions to get average
  for (int i = 0; i < no_iters; i++) {
    auto t0 = std::chrono::high_resolution_clock::now();

    std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);

    auto t1 = std::chrono::high_resolution_clock::now();

    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    total_time += elapsed_time_ms;

    for (size_t i = 0; i < pathPointers.size(); i++) path.push_back(*pathPointers[i]);
  }

  measuresPath << total_time / no_iters << "\n";
  measuresPath.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Average Delaunay Triangulations processed in %f ms.",
              (total_time / no_iters));

  return path;
}

/**
 * @brief Test function for outlier removal's efficiency.
 * Runs 100 times and calculates average duration.
 *
 * @param filename path to the file that contains the data for testing
 * @param num_outliers Number of outliers
 */
void outlierCalculations(std::string filename, int num_outliers = 0) {
  Track *track = new Track();
  track->fillTrack(filename);  // fill track with file data

  // Remove outliers no_iters times to get average
  int no_iters = 100;
  double total_time = 0;

  std::ofstream measuresPath = openWriteFile(
      "performance/exec_time/planning/planning_" + get_current_date_time_as_string() + ".csv",
      "Number of Left Cones,Number of Right Cones,Number of "
      "Outliers,Outliers Removal Execution "
      "Time,Triangulations Execution Time");

  for (int i = 0; i < no_iters; i++) {
    track->reset();
    track->fillTrack(filename);
    auto t0 = std::chrono::high_resolution_clock::now();
    track->validateCones();
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed_time_iter_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    total_time += elapsed_time_iter_ms;
  }

  measuresPath << track->getLeftConesSize() << "," << track->getRightConesSize() << ","
               << num_outliers << "," << total_time / no_iters << ",";
  measuresPath.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Outliers removed in average %f ms.",
              (total_time / no_iters));
}

std::ostream &operator<<(std::ostream &os, const PathPoint &p) {
  return os << '(' << p.getX() << ", " << p.getY() << ')';
}

/**
 * @brief simple case in which there are cones only in the left size
 * and no significant outliers
 *
 */
TEST(LocalPathPlanner, outlier_test2) {
  std::string file_path = "src/planning/tracks/outlier_test2.txt";
  Track *track = new Track();
  track->fillTrack(file_path);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track->getLeftConesSize();
  int n1_right = track->getRightConesSize();
  EXPECT_EQ(n1_left, 12);
  EXPECT_EQ(n1_right, 0);
  track->orderCones(*track->get_pointer_to_left_cones());
  track->orderCones(*track->get_pointer_to_right_cones());
  track->validateCones();
  n1_left = track->getLeftConesSize();
  n1_right = track->getRightConesSize();
  EXPECT_EQ(n1_left, 12);
  EXPECT_EQ(n1_right, 0);
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(true), 3), 2.028);
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(false), 3), 0);
  int n2_left = track->getLeftConesSize();
  int n2_right = track->getRightConesSize();
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, 0);
  EXPECT_EQ(n2_right, 0);
  // track -> logCones(true);
  // track -> logCones(false);
}

/**
 * @brief simple scenario only with left cones but with 4 significant outliers
 *
 */
TEST(LocalPathPlanner, outliers_test1) {
  std::string file_path = "src/planning/tracks/outlier_test1.txt";
  Track *track = new Track();
  track->fillTrack(file_path);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track->getLeftConesSize();
  int n1_right = track->getRightConesSize();
  EXPECT_EQ(n1_left, 36);
  EXPECT_EQ(n1_right, 0);
  track->orderCones(*track->get_pointer_to_left_cones());
  track->orderCones(*track->get_pointer_to_right_cones());
  track->validateCones();
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(true), 3), 1.342);
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(false), 3), 0);
  int n2_left = track->getLeftConesSize();
  int n2_right = track->getRightConesSize();
  EXPECT_EQ(n1_left, 36);
  EXPECT_EQ(n2_left, 36);
  EXPECT_EQ(n1_right, 0);
  EXPECT_EQ(n2_right, 0);
  // track -> logCones(true);
  // track -> logCones(false);
}

/**
 * @brief scenario with large track but few outliers
 *
 */
TEST(LocalPathPlanner, map250_out10) {
  std::string file_path = "src/planning/tracks/map_250_out10.txt";
  Track *track = new Track();
  track->fillTrack(file_path);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track->getLeftConesSize();
  int n1_right = track->getRightConesSize();
  track->orderCones(*track->get_pointer_to_left_cones());
  track->orderCones(*track->get_pointer_to_right_cones());
  track->validateCones();
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(true), 3), 3.755);
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(false), 3), 3.629);
  int n2_left = track->getLeftConesSize();
  int n2_right = track->getRightConesSize();
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
  // track -> logCones(true);
  // track -> logCones(false);
}

/**
 * @brief scenario with large track and moderate number of outliers
 *
 */
TEST(LocalPathPlanner, map250_out25) {
  std::string file_path = "src/planning/tracks/map_250_out25.txt";
  Track *track = new Track();
  track->fillTrack(file_path);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track->getLeftConesSize();
  int n1_right = track->getRightConesSize();
  track->orderCones(*track->get_pointer_to_left_cones());
  track->orderCones(*track->get_pointer_to_right_cones());
  track->validateCones();
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(true), 3), 4.188);
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(false), 3), 3.834);
  int n2_left = track->getLeftConesSize();
  int n2_right = track->getRightConesSize();
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
  // track -> logCones(true);
  // track -> logCones(false);
}

/**
 * @brief scenario with large track and abundant outliers
 *
 */
TEST(LocalPathPlanner, map250_out50) {
  std::string file_path = "src/planning/tracks/map_250_out50.txt";
  Track *track = new Track();
  track->fillTrack(file_path);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track->getLeftConesSize();
  int n1_right = track->getRightConesSize();
  track->orderCones(*track->get_pointer_to_left_cones());
  track->orderCones(*track->get_pointer_to_right_cones());
  track->validateCones();
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(true), 3), 4.099);
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(false), 3), 3.663);
  int n2_left = track->getLeftConesSize();
  int n2_right = track->getRightConesSize();
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
  // track -> logCones(true);
  // track -> logCones(false);
}

/**
 * @brief scenario with large track that barely has outliers (only 2)
 *
 */
TEST(LocalPathPlanner, map250) {
  std::string file_path = "src/planning/tracks/map_250.txt";
  Track *track = new Track();
  track->fillTrack(file_path);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track->getLeftConesSize();
  int n1_right = track->getRightConesSize();
  track->orderCones(*track->get_pointer_to_left_cones());
  track->orderCones(*track->get_pointer_to_right_cones());
  track->validateCones();
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(true), 3), 3.755);
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(false), 3), 3.755);
  int n2_left = track->getLeftConesSize();
  int n2_right = track->getRightConesSize();
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
  // track -> logCones(true);
  // track -> logCones(false);
}

/**
 * @brief unrealistic scenario with cones randomly positioned in 100 by 100
 * square
 *
 */
TEST(LocalPathPlanner, map_250_rng) {
  std::string file_path = "src/planning/tracks/map_250_rng.txt";
  Track *track = new Track();
  track->fillTrack(file_path);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track->getLeftConesSize();
  int n1_right = track->getRightConesSize();
  track->orderCones(*track->get_pointer_to_left_cones());
  track->orderCones(*track->get_pointer_to_right_cones());
  track->validateCones();
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(true), 3), 5.044);
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(false), 3), 9.759);
  int n2_left = track->getLeftConesSize();
  int n2_right = track->getRightConesSize();
  EXPECT_EQ(n1_left, n2_left);
  EXPECT_EQ(n1_right, n2_right);
  // track -> logCones(true);
  // track -> logCones(false);
}

/**
 * @brief simplest scenario with cones forming a line and one outlier
 *
 */
TEST(LocalPathPlanner, distance_next_outliers) {
  std::string file_path = "src/planning/tracks/distance_to_next.txt";
  Track *track = new Track();
  track->fillTrack(file_path);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track->getLeftConesSize();
  int n1_right = track->getRightConesSize();
  track->validateCones();
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(true), 3), 0);
  EXPECT_FLOAT_EQ(round_n(track->getMaxDistance(false), 3), 1.437);
  int n2_left = track->getLeftConesSize();
  int n2_right = track->getRightConesSize();
  EXPECT_EQ(n1_left, 0);
  EXPECT_EQ(n2_left, 0);
  EXPECT_EQ(n1_right, 15);
  EXPECT_EQ(n2_right, 15);
  // track -> logCones(true);
  // track -> logCones(false);
}

/**
 * @brief simple scenario with few points in the path
 *
 */
TEST(LocalPathPlanner, path_smooth1) {
  std::string file_path = "src/planning/tracks/path_smooth1.txt";
  PathSmoothing *new_path = new PathSmoothing();
  new_path->fillPath(file_path);
  // new_path -> logPathPoints();
  std::vector<PathPoint *> pathPointers = new_path->getPath();
  new_path->defaultSmoother(pathPointers);
  EXPECT_EQ(new_path->getPointAmount(), 111);
  // new_path -> logPathPoints();
}

/**
 * @brief more complex scenario with cones deviating from path and with
 * significant curvature
 *
 */
TEST(LocalPathPlanner, path_smooth2) {
  std::string file_path = "src/planning/tracks/path_smooth2.txt";
  PathSmoothing *new_path = new PathSmoothing();
  new_path->fillPath(file_path);
  // new_path -> logPathPoints();
  std::vector<PathPoint *> pathPointers = new_path->getPath();
  new_path->defaultSmoother(pathPointers);
  EXPECT_EQ(new_path->getPointAmount(), 381);
  // new_path -> logPathPoints();
}

struct PathPointHash {
  std::size_t operator()(const PathPoint &p) const {
    // Compute individual hash values for two data members and combine them using XOR
    std::size_t h1 = std::hash<double>()(p.getX());
    std::size_t h2 = std::hash<double>()(p.getY());

    return h1 ^ h2;
  }
};

struct PathPointEqual {
  bool operator()(const PathPoint &p1, const PathPoint &p2) const {
    return p1.getX() == p2.getX() && p1.getY() == p2.getY();
  }
};

namespace std {
template <>
struct hash<PathPoint> {
  std::size_t operator()(const PathPoint &p) const {
    // Compute individual hash values for two data members and combine them using XOR
    std::size_t h1 = std::hash<double>()(p.getX());
    std::size_t h2 = std::hash<double>()(p.getY());

    return h1 ^ h2;
  }
};
}  // namespace std

/**
 * @brief simple but realistic scenario with few cones and fwe outliers
 *
 */
TEST(LocalPathPlanner, delauney10) {
  std::string file_path = "src/planning/tracks/map_10.txt";
  Track *track = new Track();
  track->fillTrack(file_path);  // fill track with file data
  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::unordered_set<PathPoint> path;
  std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++) path.insert(*pathPointers[i]);
  std::unordered_set<PathPoint> expected{
      PathPoint(1.5, 0.0),  PathPoint(1.75, 1.0),  PathPoint(2.25, 1.4),
      PathPoint(2.5, 2.4),  PathPoint(3.0, 2.75),  PathPoint(3.5, 3.15),
      PathPoint(3.75, 3.9), PathPoint(4.25, 4.25), PathPoint(4.5, 5.0)};
  EXPECT_EQ(path.size(), expected.size());
  EXPECT_EQ(expected, path);
}

/**
 * @brief scenario: Large piece of track with no outliers
 *
 */
TEST(LocalPathPlanner, delauney3_0) {
  std::string file_path = "src/planning/tracks/map_100.txt";
  Track *track = new Track();
  track->fillTrack(file_path);  // fill track with file data
  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::vector<std::pair<double, double>> path;
  std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++) {
    path.push_back({pathPointers[i]->getX(), pathPointers[i]->getY()});
  }
  std::vector<std::pair<double, double>> expected{
      {9.264488222047865, -14.544556602878384},  {11.098962470793445, -13.973058825226456},
      {12.952185068629047, -13.453427875255297}, {14.795742958854717, -12.930775270427054},
      {16.651178604314083, -12.454028390131437}, {16.889186776896732, -45.89949503428116},
      {16.99671769430034, -49.85814622291364},   {17.059442621876435, -47.84339828855889},
      {17.487038416676924, -44.12825293739533},  {18.507199338858825, -42.493363162577126},
      {18.509420243167266, -11.996424568110662}, {19.433682520335342, -40.8702000504269},
      {20.370374395591107, -11.577823648488781}, {20.66413537971954, -39.51238691751904},
      {22.12929977842547, -38.35304645082062},   {22.248899890219143, -11.201472219259287},
      {23.6818831093663, -37.07882531651846},    {24.118678008948176, -10.85627915130934},
      {25.471744684974972, -36.15352330202951},  {26.023078470269255, -10.577395793663829},
      {27.02379989782508, -35.18842366134389},   {27.904986014644113, -10.320872468384028},
      {28.817295568373027, -34.524893133928074}, {29.83254031662907, -10.166824066409102},
      {30.552399216759277, -33.719567433006056}, {31.729470914798288, -10.017925311841005},
      {32.354565874505184, -33.197949369805684}, {33.65376421108042, -10.047864842986382},
      {34.27112812873541, -32.53544296980906},   {35.71305765436516, -10.062568050365462},
      {36.09378382946615, -32.034768902581455},  {37.633000153041266, -10.302532803732168},
      {37.91853508602949, -31.52185188970161},   {39.523423685336915, -10.581239549542051},
      {39.77312236811387, -30.921543573322403},  {41.26418338575908, -11.045350144388983},
      {41.49648095138352, -30.46226176510593},   {43.090128395784475, -11.637603386407559},
      {43.34157475000643, -29.696678914651056},  {44.74864655062539, -12.374454525417715},
      {45.245568819634066, -29.130963975856815}, {46.340542083295006, -13.256954677913221},
      {46.93280563822984, -28.245579533030934},  {47.93126862938833, -14.367109164381901},
      {48.66621796774925, -27.30004239052475},   {49.36583611023501, -15.764303165321168},
      {50.10491001300154, -26.05721953652515},   {50.662655680863125, -17.299249125396475},
      {51.06575864950613, -24.477844297198516},  {51.31161508859779, -19.01713029173498},
      {51.780739560158665, -22.715865481146395}, {51.828563433823994, -20.73905250270295}};
  path = orderVectorOfPairs(path);
  expected = orderVectorOfPairs(expected);

  EXPECT_EQ(path.size(), expected.size());
  if (path.size() == expected.size()) {
    for (size_t i = 0; i < expected.size(); i++) {
      EXPECT_LE(fabs(path[i].first - expected[i].first), 0.1);
      EXPECT_LE(fabs(path[i].second - expected[i].second), 0.1);
    }
  }
  // For some reason,  the current implementation has different
  // outputs when ran different times. All results are coherent with
  // Delaunay Triangulations but vary minimally. This should be reviewed later
}

/**
 * @brief scenario: cones in parallel straight lines that shows that cones
 * exaclty in a circumference are not counted as being inside a circle
 *
 */
TEST(LocalPathPlanner, map_test1_void) {
  std::string file_path = "src/planning/tracks/map_test1.txt";
  Track *track = new Track();
  track->fillTrack(file_path);  // fill track with file data
  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::unordered_set<PathPoint> path;
  std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++) path.insert(*pathPointers[i]);
  std::unordered_set<PathPoint> expected{{1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0},
                                         {6, 0}, {7, 0}, {8, 0}, {0, 0}};
  EXPECT_EQ(path.size(), expected.size());
  EXPECT_EQ(expected, path);
}

/**
 * @brief scenario: small track with sharp edge
 *
 */
TEST(LocalPathPlanner, map_test2) {
  std::string file_path = "src/planning/tracks/map_test2.txt";
  Track *track = new Track();
  track->fillTrack(file_path);  // fill track with file data
  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::unordered_set<PathPoint> path;
  std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++) {
    path.insert({pathPointers[i]->getX(), pathPointers[i]->getY()});
  }
  std::unordered_set<PathPoint> expected = {
      {1, 0}, {1, 1},     {1, 2}, {1.5, 3},   {1.75, 3.5}, {1.75, 3.5}, {1.75, 3.5},
      {2, 4}, {2.5, 4.5}, {3, 5}, {3.5, 4.5}, {4, 4},      {3.5, 2},    {4.5, 3.5}};
  EXPECT_EQ(path.size(), expected.size());
  EXPECT_EQ(path, expected);
}

/**
 * Extracts the size and number of outliers from a filename.
 *
 * @param filename The filename containing size and number of outliers
 * information.
 * @param size Output parameter to store the extracted size.
 * @param n_outliers Output parameter to store the extracted number of outliers.
 *
 * The filename should be in the format "map_{size}_{n_outliers}.txt", where:
 *   - "size" is an integer representing the size.
 *   - "n_outliers" is an integer representing the number of outliers.
 *
 * Example:
 *   If filename is "map_100_5.txt", size will be set to 100 and n_outliers
 * to 5.
 */
void extractInfo(const std::string_view &filenameView, int &size, int &n_outliers) {
  std::string filename(filenameView);
  size_t pos1 = filename.find("_");
  size_t pos2 = filename.find("_", pos1 + 1);
  size_t pos3 = filename.find(".", pos2 + 1);
  std::string size_str = filename.substr(pos1 + 1, pos2 - pos1 - 1);
  std::string n_outliers_str = filename.substr(pos2 + 1, pos3 - pos2 - 1);
  std::istringstream(size_str) >> size;
  std::istringstream(n_outliers_str) >> n_outliers;
}

TEST(ConeColoring, filtercones) {
  Track track;
  track.fillTrack("src/planning/tracks/track1.txt");
  std::vector<Cone *> test_cones = track.get_left_cones();
  for (Cone *c : track.get_right_cones()) {
    test_cones.push_back(c);
  }
  double gain_angle = 1.0;
  double gain_distance = 1.0;
  double gain_ncones = 1.0;
  double exponent_1 = 1.0;
  double exponent_2 = 1.0;
  double cost_max = 5000.0;
  auto cone_coloring =
      ConeColoring(gain_angle, gain_distance, gain_ncones, exponent_1, exponent_2, cost_max);
  auto initial_position = Position(30, 15);
  double radius = 5.0;
  test_cones = cone_coloring.filter_cones_by_distance(test_cones, initial_position, radius);
  std::vector<int> expected = {132, 134, 136, 145, 147};
  int count = 0;
  for (const Cone *c : test_cones) {
    EXPECT_EQ(c->getId(), expected[count]);
    auto position2 = Position(c->getX(), c->getY());
    ASSERT_LE(initial_position.distance_squared_to(position2), radius * radius);
    count++;
  }
}

TEST(ConeColoring, get_first_cones) {
  Track track;
  track.fillTrack("src/planning/tracks/track1.txt");
  std::vector<Cone *> test_cones = track.get_left_cones();
  for (Cone *c : track.get_right_cones()) {
    test_cones.push_back(c);
  }
  double gain_angle = 1.0;
  double gain_distance = 1.0;
  double gain_ncones = 1.0;
  double exponent_1 = 1.0;
  double exponent_2 = 1.0;
  double cost_max = 5000.0;
  auto cone_coloring =
      ConeColoring(gain_angle, gain_distance, gain_ncones, exponent_1, exponent_2, cost_max);
  auto initial_position = Position(30, 15);
  test_cones = cone_coloring.filter_cones_by_distance(test_cones, initial_position, 5.0);
  auto initial_car_pose = Pose(30.0, 15.0, 0);
  const Cone c1 = cone_coloring.get_initial_cone(test_cones, initial_car_pose, TrackSide::LEFT);
  EXPECT_DOUBLE_EQ(round_n(c1.getX(), 3), round_n(27.4081, 3));
  EXPECT_DOUBLE_EQ(round_n(c1.getY(), 3), round_n(17.9243, 3));
  EXPECT_EQ(c1.getId(), 0);
  const Cone c2 = cone_coloring.get_initial_cone(test_cones, initial_car_pose, TrackSide::RIGHT);
  EXPECT_DOUBLE_EQ(round_n(c2.getX(), 3), round_n(29.8945, 3));
  EXPECT_DOUBLE_EQ(round_n(c2.getY(), 3), round_n(13.0521, 3));
  EXPECT_EQ(c2.getId(), 1);
}

// test the placement of initial cones with the car oriented at 0 rad
TEST(ConeColoring, place_first_cones1) {
  Track track;
  track.fillTrack("src/planning/tracks/track1.txt");
  std::vector<Cone *> test_cones = track.get_left_cones();
  for (Cone *c : track.get_right_cones()) {
    test_cones.push_back(c);
  }
  double gain_angle = 1.0;
  double gain_distance = 1.0;
  double gain_ncones = 1.0;
  double exponent_1 = 1.0;
  double exponent_2 = 1.0;
  double cost_max = 5000.0;
  auto cone_coloring =
      ConeColoring(gain_angle, gain_distance, gain_ncones, exponent_1, exponent_2, cost_max);
  auto initial_car_pose = Pose(30.0, 15.0, 3.1416);
  cone_coloring.place_initial_cones(test_cones, initial_car_pose);
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_left_cones[0]->getX(), 3), round_n(31.8945, 3));
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_left_cones[0]->getY(), 3), round_n(13.0521, 3));
  EXPECT_EQ(cone_coloring.current_left_cones[0]->getId(), -2);
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_left_cones[1]->getX(), 3), round_n(29.8945, 3));
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_left_cones[1]->getY(), 3), round_n(13.0521, 3));
  EXPECT_EQ(cone_coloring.current_left_cones[1]->getId(), 0);
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_right_cones[0]->getX(), 3), round_n(29.4081, 3));
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_right_cones[0]->getY(), 3), round_n(17.9243, 3));
  EXPECT_EQ(cone_coloring.current_right_cones[0]->getId(), -1);
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_right_cones[1]->getX(), 3), round_n(27.4081, 3));
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_right_cones[1]->getY(), 3), round_n(17.9243, 3));
  EXPECT_EQ(cone_coloring.current_right_cones[1]->getId(), 1);
}

// test the placement of initial cones with the car oriented at pi rad
TEST(ConeColoring, place_first_cones2) {
  Track track;
  track.fillTrack("src/planning/tracks/track1.txt");
  std::vector<Cone *> test_cones = track.get_left_cones();
  for (Cone *c : track.get_right_cones()) {
    test_cones.push_back(c);
  }
  double gain_angle = 1.0;
  double gain_distance = 1.0;
  double gain_ncones = 1.0;
  double exponent_1 = 1.0;
  double exponent_2 = 1.0;
  double cost_max = 5000.0;
  auto cone_coloring =
      ConeColoring(gain_angle, gain_distance, gain_ncones, exponent_1, exponent_2, cost_max);
  auto initial_car_pose = Pose(30.0, 15.0, 0);
  cone_coloring.place_initial_cones(test_cones, initial_car_pose);
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_right_cones[0]->getX(), 3), round_n(27.8945, 3));
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_right_cones[0]->getY(), 3), round_n(13.0521, 3));
  EXPECT_EQ(cone_coloring.current_left_cones[0]->getId(), -2);
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_right_cones[1]->getX(), 3), round_n(29.8945, 3));
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_right_cones[1]->getY(), 3), round_n(13.0521, 3));
  EXPECT_EQ(cone_coloring.current_left_cones[1]->getId(), 0);
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_left_cones[0]->getX(), 3), round_n(25.4081, 3));
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_left_cones[0]->getY(), 3), round_n(17.9243, 3));
  EXPECT_EQ(cone_coloring.current_right_cones[0]->getId(), -1);
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_left_cones[1]->getX(), 3), round_n(27.4081, 3));
  EXPECT_DOUBLE_EQ(round_n(cone_coloring.current_left_cones[1]->getY(), 3), round_n(17.9243, 3));
  EXPECT_EQ(cone_coloring.current_right_cones[1]->getId(), 1);
}

TEST(ConeColoring, making_unvisited_cones1) {
  Track track;
  track.fillTrack("src/planning/tracks/track1.txt");
  std::vector<Cone *> test_cones = track.get_left_cones();
  for (Cone *c : track.get_right_cones()) {
    test_cones.push_back(c);
  }
  double gain_angle = 1.0;
  double gain_distance = 1.0;
  double gain_ncones = 1.0;
  double exponent_1 = 1.0;
  double exponent_2 = 1.0;
  double cost_max = 5000.0;
  auto cone_coloring =
      ConeColoring(gain_angle, gain_distance, gain_ncones, exponent_1, exponent_2, cost_max);
  auto cones = cone_coloring.make_unvisited_cones(test_cones);
  EXPECT_EQ((*cones).size(), test_cones.size());
  for (int i = 0; i < (int)(*cones).size(); i++) {
    EXPECT_DOUBLE_EQ((*cones)[i].first->getX(), test_cones[i]->getX());
    EXPECT_DOUBLE_EQ((*cones)[i].first->getY(), test_cones[i]->getY());
    EXPECT_EQ((*cones)[i].first->getId(), test_cones[i]->getId());
  }
}

TEST(ConeColoring, fullconecoloring1) {
  Track track;
  track.fillTrack("src/planning/tracks/track1.txt");
  std::vector<Cone *> test_cones = track.get_left_cones();
  for (Cone *c : track.get_right_cones()) {
    test_cones.push_back(c);
  }
  int c_right;
  int inc_right;
  int c_left;
  int inc_left;
  double gain_angle = 1.0;
  double gain_distance = 1.0;
  double gain_ncones = 10.0;
  double exponent_1 = 1.0;
  double exponent_2 = 1.0;
  double cost_max = 10.0;
  auto cone_coloring =
      ConeColoring(gain_angle, gain_distance, gain_ncones, exponent_1, exponent_2, cost_max);
  auto initial_car_pose = Pose(30.0, 15.0, 3.14);
  cone_coloring.color_cones(test_cones, initial_car_pose, 5.0);
  test_cone_coloring(cone_coloring, c_left, c_right, inc_left, inc_right);
  EXPECT_EQ(c_left, 128);
  EXPECT_EQ(c_right, 140);
  EXPECT_EQ(inc_left, 0);
  EXPECT_EQ(inc_right, 0);
}

TEST(ConeColoring, fullconecoloring2) {
  double gain_angle = 8.7;
  double gain_distance = 15;
  double gain_ncones = 8.7;
  double exponent_1 = 0.7;
  double exponent_2 = 1.7;
  double cost_max = 40;
  for (int i = 1; i < 21; i++) {
    Track track;
    std::string file_name = "gtruths/tracks/eufs/track" + std::to_string(i) + "/converted_track" +
                            std::to_string(i) + ".txt";
    track.fillTrack(file_name);
    std::vector<Cone *> test_cones = track.get_left_cones();
    for (Cone *c : track.get_right_cones()) {
      test_cones.push_back(c);
    }
    int c_right;
    int inc_right;
    int c_left;
    int inc_left;
    auto cone_coloring =
        ConeColoring(gain_angle, gain_distance, gain_ncones, exponent_1, exponent_2, cost_max);
    auto initial_car_pose = Pose(0.0, 0.0, 0.0);
    cone_coloring.color_cones(test_cones, initial_car_pose, 5.0);
    test_cone_coloring(cone_coloring, c_left, c_right, inc_left, inc_right);
    // for (const Cone* c : cone_coloring.current_left_cones) {
    //   std::cout << "(" << c->getX() << "," << c->getY() << "),";
    // }
    // std::cout << std::endl;
    // for (const Cone* c : cone_coloring.current_right_cones) {
    //   std::cout << "(" << c->getX() << "," << c->getY() << "),";
    // }
    // std::cout << std::endl;
    // std::cout << "Track" << std::to_string(i) << ": c_left : " << c_left << " c_right: " <<
    // c_right << " inc_left: " << inc_left << " inc_right: " << inc_right << std::endl;
  }
}
