#include "test_utils/utils.hpp"

/**
 * @brief simple but realistic scenario with few cones and few outliers
 *
 */
TEST(PathCalculation, delauney10) {
  std::string file_path = "src/planning/tracks/map_10.txt";
  auto track = track_from_file(file_path);

  auto path_calculation = PathCalculation();
  std::vector<PathPoint> generated_points = path_calculation.process_delaunay_triangulations(track);

  std::unordered_set<PathPoint, PathPointHash, PathPointEqual> expected{
      PathPoint(3.5, 3.15), PathPoint(1.75, 1.0),  PathPoint(2.25, 1.4),
      PathPoint(4.5, 5),    PathPoint(1.5, 0),     PathPoint(3, 2.75),
      PathPoint(2.5, 2.4),  PathPoint(4.25, 4.25), PathPoint(3.75, 3.9)};

  for (const auto &point : generated_points) {
    EXPECT_TRUE(expected.find(point) != expected.end());
  }

  EXPECT_EQ(generated_points.size(), expected.size());
}

/**
 * @brief scenario: Large piece of track with no outliers
 *
 */
TEST(PathCalculation, delauney3_0) {
  std::string file_path = "src/planning/tracks/map_100.txt";
  auto track = track_from_file(file_path);
  std::vector<std::pair<double, double>> path;

  auto path_calculation = PathCalculation();
  std::vector<PathPoint> path_points = path_calculation.process_delaunay_triangulations(track);

  for (size_t i = 0; i < path_points.size(); i++) {
    path.push_back({path_points[i].position.x, path_points[i].position.y});
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
TEST(PathCalculation, map_test1_void) {
  std::string file_path = "src/planning/tracks/map_test1.txt";
  auto track = track_from_file(file_path);

  auto path_calculation = PathCalculation();
  std::unordered_set<PathPoint, PathPointHash, PathPointEqual> path;
  std::vector<PathPoint> path_points = path_calculation.process_delaunay_triangulations(track);
  for (size_t i = 0; i < path_points.size(); i++) path.insert(path_points[i]);
  std::unordered_set<PathPoint, PathPointHash, PathPointEqual> expected{
      {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}, {0, 0}};
  EXPECT_EQ(path.size(), expected.size());
  EXPECT_EQ(expected, path);
}

/**
 * @brief scenario: small track with sharp edge
 *
 */
TEST(PathCalculation, map_test2) {
  std::string file_path = "src/planning/tracks/map_test2.txt";
  auto track = track_from_file(file_path);

  auto path_calculation = PathCalculation();
  std::unordered_set<PathPoint, PathPointHash, PathPointEqual> path;
  std::vector<PathPoint> path_points = path_calculation.process_delaunay_triangulations(track);
  for (size_t i = 0; i < path_points.size(); i++) path.insert(path_points[i]);
  std::unordered_set<PathPoint, PathPointHash, PathPointEqual> expected = {
      {1, 0}, {1, 1},     {1, 2}, {1.5, 3},   {1.75, 3.5}, {1.75, 3.5}, {1.75, 3.5},
      {2, 4}, {2.5, 4.5}, {3, 5}, {3.5, 4.5}, {4, 4},      {3.5, 2},    {4.5, 3.5}};
  EXPECT_EQ(path.size(), expected.size());
  EXPECT_EQ(path, expected);
}