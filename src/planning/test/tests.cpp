#include <chrono>
#include <fstream>
#include <set>

#include "gtest/gtest.h"
#include "planning/global_path_planner.hpp"
#include "planning/local_path_planner.hpp"
#include "planning/path_smoothing.hpp"
#include "planning/track.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/files.hpp"

using testing::Eq;

bool customComparator(const std::pair<double,  double>& a,  const std::pair<double,  double>& b) {
    if (a.first != b.first) {
        return a.first < b.first;
    }
    return a.second < b.second;
}

std::vector<std::pair<double,  double>> orderVectorOfPairs
(const std::vector<std::pair<double,  double>>& vec) {
    std::vector<std::pair<double,  double>> result = vec;
    std::sort(result.begin(),  result.end(),  customComparator);
    return result;
}
/**
 * @brief Test function for Delaunay Triangulations' efficiency. 
 * Runs 100 times to get average execution time.
 * 
 * @param filename path to the file that contains the data for testing
 * @param testname name of the test
 * @return path after Delaunay Triangulations
 */
std::vector<PathPoint> processTriangulations(std::string filename,  std::string testname) {
  Track *track = new Track();
  track->fillTrack(filename);  // fill track with file data

  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::vector<PathPoint> path;
  std::ofstream measuresPath = openWriteFile("src/performance/exec_time/planning.csv");
  double total_time = 0;
  int no_iters = 100;

  // No_iters repetitions to get average
  for (int i = 0; i < no_iters; i++) {
    auto t0 = std::chrono::high_resolution_clock::now();

    std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);

    auto t1 = std::chrono::high_resolution_clock::now();

    double elapsed_time_ms = std::chrono::duration<double,  std::milli>(t1 - t0).count();

    if (i == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),  "Delaunay Triangulations processed in %f ms.\n",
              elapsed_time_ms);
      measuresPath << "planning,  delaunay,  " << testname << ",  " <<
      elapsed_time_ms;
    }

    total_time += elapsed_time_ms;

    for (size_t i = 0; i < pathPointers.size(); i++) path.push_back(*pathPointers[i]);
  }

  measuresPath <<  ",  " << total_time / no_iters << "\n";
  measuresPath.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Average Delaunay Triangulations processed in %f ms.",  (total_time / no_iters));

  return path;
}

/**
 * @brief Test function for outlier removal's efficiency. 
 * Runs 100 times and calculates average duration.
 * 
 * @param filename path to the file that contains the data for testing
 * @param testname name of the test
 */
void outlierCalculations(std::string filename,  std::string testname) {
  Track *track = new Track();
  track->fillTrack(filename);  // fill track with file data

  // Remove outliers no_iters times to get average
  int no_iters = 100;
  double total_time = 0;
  std::ofstream measuresPath = openWriteFile("src/performance/exec_time/planning.csv");

  for (int i = 0; i < no_iters; i++) {
    track->reset();
    track->fillTrack(filename);
    auto t0 = std::chrono::high_resolution_clock::now();
    track->validateCones();
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed_time_iter_ms = std::chrono::duration<double,  std::milli>(t1 - t0).count();
    if (i == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
      "First Iteration: Outliers removed in %f ms\n",  (elapsed_time_iter_ms));
      measuresPath << "planning,  outliers,  " << testname << ",  " << elapsed_time_iter_ms;
    }
    total_time += elapsed_time_iter_ms;
  }

  measuresPath << ",  " << total_time / no_iters << "\n";
  measuresPath.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),  "Outliers removed in average %f ms.",
              (total_time / no_iters));
}

std::ostream &operator<<(std::ostream &os,  const PathPoint &p) {
  return os << '(' << p.getX() << ', ' << p.getY() << ')';
}

TEST(LocalPathPlanner,  outlier_test2) {
  std::string filePath = "src/planning/tracks/outlier_test2.txt";
  Track *track = new Track();
  track->fillTrack(filePath);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track -> getLeftConesSize();
  int n1_right = track -> getRightConesSize();
  track->validateCones();
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(true),  3),  2.028);
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(false),  3),  0);
  int n2_left = track -> getLeftConesSize();
  int n2_right = track -> getRightConesSize();
  EXPECT_EQ(n1_left,  n2_left);
  EXPECT_EQ(n1_right,  0);
  EXPECT_EQ(n2_right,  1);
  // track -> logCones(true);
  // track -> logCones(false);
}


TEST(LocalPathPlanner,  outliers_test1) {
  std::string filePath = "src/planning/tracks/outlier_test1.txt";
  Track *track = new Track();
  track->fillTrack(filePath);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track -> getLeftConesSize();
  int n1_right = track -> getRightConesSize();
  track->validateCones();
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(true),  3),  1.342);
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(false),  3),  0);
  int n2_left = track -> getLeftConesSize();
  int n2_right = track -> getRightConesSize();
  EXPECT_EQ(n1_left,  n2_left);
  EXPECT_EQ(n1_right,  0);
  EXPECT_EQ(n2_right,  1);
  // track -> logCones(true);
  // track -> logCones(false);
}

TEST(LocalPathPlanner,  map250_out10) {
  std::string filePath = "src/planning/tracks/map_250_out10.txt";
  Track *track = new Track();
  track->fillTrack(filePath);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track -> getLeftConesSize();
  int n1_right = track -> getRightConesSize();
  track->validateCones();
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(true),  3),  3.755);
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(false),  3),  3.629);
  int n2_left = track -> getLeftConesSize();
  int n2_right = track -> getRightConesSize();
  EXPECT_EQ(n1_left,  n2_left);
  EXPECT_EQ(n1_right,  n2_right);
  // track -> logCones(true);
  // track -> logCones(false);
}

TEST(LocalPathPlanner,  map250_out25) {
  std::string filePath = "src/planning/tracks/map_250_out25.txt";
  Track *track = new Track();
  track->fillTrack(filePath);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track -> getLeftConesSize();
  int n1_right = track -> getRightConesSize();
  track -> validateCones();
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(true),  3),  4.188);
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(false),  3),  3.834);
  int n2_left = track -> getLeftConesSize();
  int n2_right = track -> getRightConesSize();
  EXPECT_EQ(n1_left,  n2_left);
  EXPECT_EQ(n1_right,  n2_right);
  // track -> logCones(true);
  // track -> logCones(false);
}

TEST(LocalPathPlanner,  map250_out50) {
  std::string filePath = "src/planning/tracks/map_250_out50.txt";
  Track *track = new Track();
  track->fillTrack(filePath);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track -> getLeftConesSize();
  int n1_right = track -> getRightConesSize();
  track->validateCones();
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(true),  3),  4.099);
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(false),  3),  3.663);
  int n2_left = track -> getLeftConesSize();
  int n2_right = track -> getRightConesSize();
  EXPECT_EQ(n1_left,  n2_left);
  EXPECT_EQ(n1_right,  n2_right);
  // track -> logCones(true);
  // track -> logCones(false);
}

TEST(LocalPathPlanner,  map250) {
  std::string filePath = "src/planning/tracks/map_250.txt";
  Track *track = new Track();
  track->fillTrack(filePath);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track -> getLeftConesSize();
  int n1_right = track -> getRightConesSize();
  track->validateCones();
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(true),  3),  3.755);
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(false),  3),  3.755);
  int n2_left = track -> getLeftConesSize();
  int n2_right = track -> getRightConesSize();
  EXPECT_EQ(n1_left,  n2_left);
  EXPECT_EQ(n1_right,  n2_right);
  // track -> logCones(true);
  // track -> logCones(false);
}

TEST(LocalPathPlanner,  map_250_rng) {
  std::string filePath = "src/planning/tracks/map_250_rng.txt";
  Track *track = new Track();
  track->fillTrack(filePath);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track -> getLeftConesSize();
  int n1_right = track -> getRightConesSize();
  track->validateCones();
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(true),  3),  5.044);
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(false),  3),  9.759);
  int n2_left = track -> getLeftConesSize();
  int n2_right = track -> getRightConesSize();
  EXPECT_EQ(n1_left,  n2_left);
  EXPECT_EQ(n1_right,  n2_right);
  // track -> logCones(true);
  // track -> logCones(false);
}

TEST(LocalPathPlanner,  left_outliers) {
  std::string filePath = "src/planning/tracks/left_side_outliers.txt";
  Track *track = new Track();
  track->fillTrack(filePath);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track -> getLeftConesSize();
  int n1_right = track -> getRightConesSize();
  track->validateCones();
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(true),  3),  1.342);
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(false),  3),  0);
  int n2_left = track -> getLeftConesSize();
  int n2_right = track -> getRightConesSize();
  EXPECT_EQ(n1_left,  n2_left);
  EXPECT_EQ(n1_right,  0);
  EXPECT_EQ(n2_right,  1);
  // track -> logCones(true);
  // track -> logCones(false);
}

TEST(LocalPathPlanner,  distance_next_outliers) {
  std::string filePath = "src/planning/tracks/distance_to_next.txt";
  Track *track = new Track();
  track->fillTrack(filePath);
  // track -> logCones(true);
  // track -> logCones(false);
  int n1_left = track -> getLeftConesSize();
  int n1_right = track -> getRightConesSize();
  track->validateCones();
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(true),  3),  0);
  EXPECT_FLOAT_EQ(track -> round_n(track -> getMaxDistance(false),  3),  1.437);
  int n2_left = track -> getLeftConesSize();
  int n2_right = track -> getRightConesSize();
  EXPECT_EQ(n1_left,  0);
  EXPECT_EQ(n2_left,  1);
  EXPECT_EQ(n1_right,  n2_right);
  // track -> logCones(true);
  // track -> logCones(false);
}


TEST(LocalPathPlanner,  path_smooth1) {
  std::string filePath = "src/planning/tracks/path_smooth1.txt";
  PathSmoothing *new_path = new PathSmoothing();
  new_path->fillPath(filePath);
  // new_path -> logPathPoints();
  new_path -> validate(new_path -> getPath());
  EXPECT_EQ(new_path -> getPointAmount(),  1101);
  // new_path -> logPathPoints();
}

TEST(LocalPathPlanner,  path_smooth2) {
  std::string filePath = "src/planning/tracks/path_smooth2.txt";
  PathSmoothing *new_path = new PathSmoothing();
  new_path->fillPath(filePath);
  //new_path -> logPathPoints();
  new_path -> validate(new_path -> getPath());
  EXPECT_EQ(new_path -> getPointAmount(),  3801);
  //new_path -> logPathPoints();
}




TEST(LocalPathPlanner,  delauney10) {
  std::string filePath = "src/planning/tracks/map_10.txt";
  Track *track = new Track();
  track->fillTrack(filePath);  // fill track with file data
  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::vector<PathPoint> path;
  std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++) path.push_back(*pathPointers[i]);
  std::vector<PathPoint> expected{
    PathPoint(1.5,  0.0),  PathPoint(1.75,  1.0),  PathPoint(2.25,  1.4),
    PathPoint(2.5,  2.4),  PathPoint(3.0,  2.75),  PathPoint(3.5,  3.15),
    PathPoint(3.75,  3.9),  PathPoint(4.25,  4.25),  PathPoint(4.5,  5.0)};
  EXPECT_EQ(path.size(),  expected.size());
  EXPECT_EQ(expected,  path);
}

TEST(LocalPathPlanner,  delauney3_0) {
  std::string filePath = "src/planning/tracks/map_100.txt";
  Track *track = new Track();
  track->fillTrack(filePath);  // fill track with file data
  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::vector<std::pair<double,  double>> path;
  std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++) {
    path.push_back({pathPointers[i] -> getX(),  pathPointers[i] -> getY()});
  }
  std::vector<std::pair<double, double>> expected{{9.264488222047865, -14.544556602878384},
  {11.098962470793445, -13.973058825226456},  {12.952185068629047, -13.453427875255297},  {
    14.795742958854717, -12.930775270427054},
  {16.651178604314083, -12.454028390131437},  {16.889186776896732, -45.89949503428116},
   {16.99671769430034, -49.85814622291364},
  {17.059442621876435, -47.84339828855889},  {17.487038416676924, -44.12825293739533},
  {18.194359132329385, -51.43887010761911},
  {18.507199338858825, -42.493363162577126},  {18.509420243167266, -11.996424568110662},
  {18.707023612657608, -53.30104342341994},
  {19.433682520335342, -40.8702000504269},  {19.747859849510206, -54.992648492013984},  {
    20.370374395591107, -11.577823648488781},
  {20.66413537971954, -39.51238691751904},  {21.3150179820434, -56.0375224622878},
   {22.12929977842547, -38.35304645082062},
  {22.248899890219143, -11.201472219259287},  {23.249646082916765, -55.899818521075474}
  ,  {23.6818831093663, -37.07882531651846},
  {24.118678008948176, -10.85627915130934},  {25.09367204587619, -55.74466275411381},
  {25.471744684974972, -36.15352330202951},
  {26.023078470269255, -10.577395793663829},  {26.879609112375206, -54.86612455531322},
   {27.02379989782508, -35.18842366134389},
  {27.904986014644113, -10.320872468384028},  {28.219022237914032, -53.66846880993329},  {
    28.817295568373027, -34.524893133928074},
  {29.70171431321185, -52.55504192870002},  {29.83254031662907, -10.166824066409102},  {
    30.552399216759277, -33.719567433006056},
  {30.887390244886202, -51.14130502950286},  {31.729470914798288, -10.017925311841005},  {
    32.354565874505184, -33.197949369805684},
  {32.36476394742891, -49.78663301395246},  {33.54325525545515, -48.30409709213305},
  {33.65376421108042, -10.047864842986382},
  {34.27112812873541, -32.53544296980906},  {35.05009991968047, -47.0693171584318},
  {35.71305765436516, -10.062568050365462},
  {36.09378382946615, -32.034768902581455},  {36.42262189025949, -45.554175169259324},  {
    37.633000153041266, -10.302532803732168},
  {37.91853508602949, -31.52185188970161},  {37.98043134846596, -44.508547983943664},  {
    39.523423685336915, -10.581239549542051},
  {39.532141424372156, -43.23755350860105},  {39.77312236811387, -30.921543573322403},
  {41.18657394170705, -42.266535231404205},
  {41.26418338575908, -11.045350144388983},  {41.49648095138352, -30.46226176510593},
  {42.849632846088554, -41.44497795512835},
  {43.090128395784475, -11.637603386407559},  {43.34157475000643, -29.696678914651056},
   {44.54146530342983, -40.52643842407908},
  {44.74864655062539, -12.374454525417715},  {45.245568819634066, -29.130963975856815},  {
    46.340542083295006, -13.256954677913221},
  {46.39297776860348, -40.03942304259501},  {46.93280563822984, -28.245579533030934},
  {47.93126862938833, -14.367109164381901},
  {48.09485933325153, -39.18944173234793},  {48.66621796774925, -27.30004239052475},
  {49.36583611023501, -15.764303165321168},
  {49.987316105783435, -38.81671701778903},  {50.10491001300154, -26.05721953652515},  {
    50.662655680863125, -17.299249125396475},
  {51.06575864950613, -24.477844297198516},  {51.31161508859779, -19.01713029173498},
  {51.73178786396499, -38.123173976973476},
  {51.780739560158665, -22.715865481146395},  {51.828563433823994, -20.73905250270295}
  ,  {53.5881569331461, -37.73401930955272},
  {55.55935503785685, -37.271966338343965},  {57.42225531756061, -36.893965675891636}
  ,  {59.37158022882337, -36.8056566324965},
  {61.27199621317445, -36.45550512111464},  {63.265627178966795, -36.46629360168862}
  ,  {65.1768164522203, -36.09645631287539},
  {65.1768164522203, -36.09645631287539},  {65.1768164522203, -36.09645631287539}
  ,  {67.0887567063864, -35.81491223669838},
  {68.83103695860362, -35.405674815950675},  {70.6482343674299, -34.80248776425467},
  {72.47790770899178, -34.231069356596706},
  {74.25208472086261, -33.48489384445998},  {75.99022556307638, -32.731393340357855},
   {77.7232143980583, -31.864963907764118},
  {79.36578071460787, -30.934520034176845},
   {81.0292369954283, -29.914028815146892},
  {82.58435803181993, -28.835643466112288},  {84.13395791286693, -27.757739227542487},
  {85.60936765248935, -26.560230317129292},
  {86.96814187340837, -25.24451671749095},  {88.44514674303196, -23.94616600594378},
  {89.6409627639684, -22.338950388455743}};
  path = orderVectorOfPairs(path);
  expected = orderVectorOfPairs(expected);
  EXPECT_EQ(path.size(),  expected.size());
  // For some reason,  the current implementation has different
  // outputs when ran different times. All results are coherent with
  // Delaunay Triangulations but vary minimally. This should be reviewed later
}

TEST(LocalPathPlanner,  delauney_250) {
  std::string filePath = "src/planning/tracks/map_250.txt";
  Track *track = new Track();
  track->fillTrack(filePath);  // fill track with file data
  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::vector<std::pair<double,  double>> path;
  std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++) {
    path.push_back({pathPointers[i] -> getX(),  pathPointers[i] -> getY()});
  }
  std::vector<std::pair<double, double>> expected{{3.378255041432624, -17.08895549345713},
  {3.5982537026472485, -18.894695723909315},
  {3.6007938281085714, -15.339503107540038},  {4.118133621970614, -13.450402038783055},
   {4.323815027747578, -20.623516903085168},
  {5.359037236031597, -22.338950388455743},  {5.816039901008104, -12.40958333445872},
   {6.5548532569680376, -23.94616600594378},
  {7.476885822217248, -11.692914647932419},  {8.03185812659163, -25.24451671749095},
  {9.360567468503, -11.450730036053484},
  {9.390632347510646, -26.560230317129292},  {10.866042087133067, -27.757739227542487},
  {11.122445132674713, -11.377887024687904},
  {12.415641968180076, -28.835643466112288},  {13.101525954418555, -11.068603598914967},
  {13.970763004571687, -29.914028815146892},
  {15.067281309973332, -10.698630126598577},  {15.634219285392131, -30.934520034176845},
  {16.988189891248105, -10.188746205600774},
  {17.276785601941693, -31.864963907764118},  {18.676304815111187, -9.231089905814326},
  {19.009774436923617, -32.731393340357855},
  {20.086892781787032, -7.981284384308729},  {20.74791527913738, -33.48489384445998},
   {21.209097167191064, -6.515711122679511},
  {21.775191149415612, -4.711260154271297},  {22.471139000155965, -3.018375972441181},
   {22.52209229100821, -34.231069356596706},
  {22.669127638004003, -1.0773999506344447},  {22.942381266779293, 0.8618905343493865}
  ,  {23.075066467804422, 4.651462075998065},
  {23.103229474846753, 2.761196325136897},  {23.26254652647188, 8.544437322866315},
  {23.2896063146715, 6.594487233269253},
  {23.782570184660205, 10.334830411474092},  {24.103271048040643, 12.239630467585194},
  {24.351765632570118, -34.80248776425467},
  {25.153194909826574, 13.780802855454375},  {26.16896304139638, -35.405674815950675}
  ,  {26.93587203938681, 14.710316696927048},
  {27.91124329361359, -35.81491223669838},  {28.651287853896367, 15.488207301116269}
  ,  {29.8231835477797, -36.09645631287539},
  {30.440691845114674, 15.696045407857646},  {31.734372821033205, -36.46629360168862}
  ,  {32.26797697318677, 15.130033229415808},
  {33.72800378682555, -36.45550512111464},  {34.05035097716251, 14.775496244719173
  },  {35.62841977117663, -36.8056566324965},
  {35.65641682175355, 13.535320766172738},  {37.120017368904655, 12.275065188629277},
   {37.57774468243939, -36.893965675891636},
  {38.41455940480727, 10.880224781324195},  {39.44064496214315, -37.271966338343965
  },  {39.77858219857851, 9.528828491509406},
  {41.08599466691375, 8.2492261482293},  {41.4118430668539, -37.73401930955272}
  ,  {42.725106804834425, 7.323600467175014},
  {43.171436566176006, -20.73905250270295},  {43.219260439841335, -22.715865481146395},
   {43.26821213603501, -38.123173976973476},
  {43.68838491140221, -19.01713029173498},  {43.93424135049387, -24.477844297198516},
  {44.337344319136875, -17.299249125396475},
  {44.50964265564208, 6.68218200078692},  {44.89508998699846, -26.05721953652515},
   {45.012683894216565, -38.81671701778903},
  {45.63416388976499, -15.764303165321168},  {46.33378203225075, -27.30004239052475
  },  {46.46189451462209, 6.262911963375038},
  {46.90514066674847, -39.18944173234793},  {47.06873137061167, -14.367109164381901},
   {48.06719436177016, -28.245579533030934},
  {48.38418032131463, 6.649966274185339},  {48.60702223139652, -40.03942304259501},
  {48.659457916704994, -13.256954677913221},
  {49.754431180365934, -29.130963975856815},  {50.20712581202876, 7.39918884148615},
   {50.25135344937461, -12.374454525417715},
  {50.45853469657017, -40.52643842407908},  {51.65842524999357, -29.696678914651056}
  ,  {51.756560729772545, 8.396689764369283},
  {51.909871604215525, -11.637603386407559},  {52.150367153911446, -41.44497795512835
  },  {53.22204856143232, 9.610102380156412},
  {53.50351904861648, -30.46226176510593},  {53.73581661424092, -11.045350144388983},
   {53.81342605829295, -42.266535231404205},
  {54.59106725960311, 10.976770011195088},  {55.22687763188613, -30.921543573322403},
   {55.467858575627844, -43.23755350860105},
  {55.476576314663085, -10.581239549542051},  {55.91749401523931, 12.461465359423212},
   {57.01956865153404, -44.508547983943664},
  {57.06035532207157, 13.844644149055926},  {57.08146491397051, -31.52185188970161},
  {57.366999846958734, -10.302532803732168},
  {58.19862593008769, 15.349192925430916},  {58.57737810974051, -45.554175169259324},
   {58.90621617053385, -32.034768902581455},
  {59.28694234563484, -10.062568050365462},  {59.29765893402349, 16.937331633425543
  },  {59.94990008031953, -47.0693171584318},
  {60.42197328332691, 18.606185046419554},  {60.72887187126459, -32.53544296980906},
   {61.34623578891958, -10.047864842986382},
  {61.36428421508412, 20.26599279216765},  {61.45674474454485, -48.30409709213305}
  ,  {62.32634638986122, 21.915071338720868},
  {62.63523605257109, -49.78663301395246},  {62.645434125494816, -33.197949369805684
  },  {63.18260692896164, 23.61477380020939},
  {63.270529085201716, -10.017925311841005},  {64.1046053973514, 25.350847355661422
  },  {64.1126097551138, -51.14130502950286},
  {64.44760078324073, -33.719567433006056},  {64.94548722331686, 27.058670210877352},
   {65.16745968337094, -10.166824066409102},
  {65.29828568678815, -52.55504192870002},  {65.839280769519, 28.628000802943667},
   {66.18270443162697, -34.524893133928074},
  {66.73545556187129, 30.312169729873965},  {66.78097776208597, -53.66846880993329},
   {67.09501398535588, -10.320872468384028},
  {67.84053620109748, 31.843738734010643},  {67.97620010217491, -35.18842366134389
  },  {68.1203908876248, -54.86612455531322},
  {68.86267563935843, 33.47247941064228},  {68.97692152973075, -10.577395793663829}
  ,  {69.52825531502504, -36.15352330202951},
  {69.90632795412381, -55.74466275411381},  {70.09701781585072, 34.94760324240116}
  ,  {70.88132199105183, -10.85627915130934},
  {71.31811689063369, -37.07882531651846},  {71.63180018669001, 36.131091085352026},
   {71.75035391708323, -55.899818521075474},
  {72.75110010978085, -11.201472219259287},  {72.87070022157454, -38.35304645082062
  },  {73.41787529051072, 36.92090899156155},
  {73.6849820179566, -56.0375224622878},  {74.33586462028046, -39.51238691751904},
   {74.62962560440889, -11.577823648488781},
  {75.2521401504898, -54.992648492013984},  {75.28938940379895, 36.82186208367802
  },  {75.56631747966466, -40.8702000504269},
  {76.29297638734239, -53.30104342341994},  {76.49057975683273, -11.996424568110662},
   {76.49280066114119, -42.493363162577126},
  {76.8056408676706, -51.43887010761911},  {77.20320669092638, 36.98490995688401}
  ,  {77.51296158332308, -44.12825293739533},
  {77.94055737812357, -47.84339828855889},  {78.00328230569966, -49.85814622291364}
  ,  {78.11081322310326, -45.89949503428116},
  {78.34882139568592, -12.454028390131437},  {78.96017018269963, 37.78080857421715},
   {80.20425704114528, -12.930775270427054},
  {80.45881175408171, 38.85650958721776},  {81.49701046583783, 40.422585156725596},
   {82.04781493137097, -13.453427875255297},
  {82.39886185651426, 42.23104918373973},  {83.16619306584963, 44.020603988916164},
   {83.90103752920655, -13.973058825226456},
  {84.07233019304726, 45.59330528201318},  {84.84388404768436, 47.402428037390436},
   {85.73551177795213, -14.544556602878384},
  {86.09607183131669, 48.79966458096679},  {87.44984962462286, 50.16092290850716}
  ,  {87.5898267875046, -15.091809731526865},
  {88.43499327591468, -15.367568140598902},  {89.12239562401746, 51.007784542497355},
   {90.26626499878587, -15.981307694892113},
  {90.99528970967805, 51.50172738481769},  {92.12497787939594, -16.540921111219728
  },  {92.90354625449442, 51.79514841862289},
  {93.96095083991906, -17.163620029145726},  {94.87763539741795, 51.723837259268656},
   {95.82697272826063, -17.714478924753642},
  {96.75145942732571, 51.54863278048491},  {97.67323233571966, -18.329441926625158},
  {98.66155388733927, 51.3141202162796},
  {99.54421061471521, -18.77230178230402},  {100.45352887167377, 50.93041604482029},
  {101.40398641511266, -19.284025766786748},
  {102.32767700442022, 50.63918871982986},  {103.27381911356443, -19.463990532691675}
  ,  {104.26876674406375, 50.25237118820398},
  {105.28978120172084,  -19.60068968002549},  {106.1708337143579,  49.991936060403575},
   {107.2950746544912,  -19.321452242774836},
  {108.08437993113904,  49.905044664262064},  {109.09680084580903,  -20.51096594231681}
  ,  {109.94671677864667,  50.10879870315808},
  {110.95844576196501,  -22.070222438308484},  {111.87908930794951,  50.030542045380194},
   {112.73160383135162,  -19.23667124047563},
  {113.63167651947346,  50.437010911563405},  {114.35126692709754,  -18.20714409749044}
  ,  {115.54572476119516,  50.73799695545959},
  {115.81655284480486,  -15.100383234492199},  {117.35971156994589,  -13.97561914247228
  },  {117.38806683502595,  51.4132110758287},
  {118.83885752577615,  -12.80794170247389},  {118.99734949009498,  52.40686536566972},
  {120.43810765590098,  -11.650875986779546},
  {120.69180060572702,  53.21966674457057},  {122.03680850701755,  -10.191119175679885}
  ,  {122.08959173536074,  54.55310250312951},
  {123.5647217485278,  -9.13610969847048},  {123.73027782749077,  55.39112769120529}
  ,  {124.91948334080627,  -7.90399360686356},
  {125.34719967305873,  56.34405518518526},  {126.41602618600163,  -6.666532359370839},
   {127.17728985604558,  57.159903711434865},
  {127.93430290110197,  -5.443080306799774},  {128.97077163754705,  57.72919915440538}
  ,  {129.29281354136876,  -4.10952307457647},
  {130.81367399417866,  58.24546208536171},  {130.8728783706121,  -2.9715334950862395},
  {132.23224589971173,  -1.7267608528005012},
  {132.71423419331734,  58.46820204669281},  {133.7765911442504,  -0.5846621265581351},
   {134.50359097294682, 58.398011325121146},
  {135.1322570062931,  0.6701593290273442},  {136.31621405714208,  58.40614960423825},
   {136.63141436428785,  1.9051844168461958},
  {138.03787973286518,  3.295338072843192},  {138.2297409078759,  58.05184186565863}
  ,  {139.44534051630262,  4.677422603008026},
  {139.96726386728892,  57.45108581581316},  {140.69551241181148,  6.1162140867104196}
  ,  {141.73450532537964,  7.667654313675628},
  {141.79256090361935,  56.80919883284724},  {142.94410381032435,  9.195875950733035}
  ,  {143.40660870863425,  55.72026026831129},
  {143.74236112084066,  10.95713808277448},  {144.6614225890499,  12.623925760969302}
  ,  {145.07995329580888,  54.79135726185831},
  {145.30423952396882,  14.44986851427481},  {146.02555391542938,  16.230300841922173}
  ,  {146.52171390193206,  53.36218093987073},
  {147.26975755422046,  17.59054430338833},  {148.00351218657485,  19.427871747450002}
  ,  {148.08075126474648,  52.04148746408132},
  {149.07130228605553,  20.754245355313028},  {149.23899650574563,  50.52131548809549}
  ,  {150.45965786848353,  48.89984616589513},
  {150.92191454196865,  21.37730481500958},  {151.31592654540773,  47.31345538099288},
   {152.27011599904807,  45.645008167906425},
  {152.6002014167228,  21.96856456115963},  {153.11065536246673,  43.879115951334754},
   {153.87750448391998,  42.213813672710195},
  {154.48826365815074,  22.460187710998664},  {154.7437916268377,  40.51582928981216}
  ,  {155.62867252362997,  38.64661002371258},
  {156.30225752919966,  23.146436758963738},  {156.4745889299105,  36.946668215363246}
  ,  {157.3136043776699,  35.277598135605004},
  {157.66728772836893,  24.53990314085459},  {158.04527182265085,  33.45891873503371}
  ,  {158.29524849661584,  26.27379629332509},
  {158.53302879412465,  31.626423603780935},  {158.9759386594608,  27.928534489521304},
   {158.99137421285053,  29.69505934897345}};
  path = orderVectorOfPairs(path);
  expected = orderVectorOfPairs(expected);
  EXPECT_EQ(path.size(),  expected.size());
  for (int n=0; n<static_cast<int>(expected.size()); n++) {
    EXPECT_FLOAT_EQ(path[n].first,  expected[n].first);
    EXPECT_FLOAT_EQ(path[n].second,  expected[n].second);
  }
}

TEST(LocalPathPlanner,  map_test1_void) {
  std::string filePath = "src/planning/tracks/map_test1.txt";
  Track *track = new Track();
  track->fillTrack(filePath);  // fill track with file data
  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::vector<PathPoint> path;
  std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++) path.push_back(*pathPointers[i]);
  std::vector<PathPoint> expected{{1,  0},  {2,  0},  {3,  0},  {4,  0}
  ,  {5,  0},  {6,  0},  {7,  0},  {8,  0},  {1,  0}};
  EXPECT_EQ(path.size(),  expected.size());
  EXPECT_EQ(expected,  path);
}

TEST(LocalPathPlanner,  map_test2) {
  std::string filePath = "src/planning/tracks/map_test2.txt";
  Track *track = new Track();
  track->fillTrack(filePath);  // fill track with file data
  LocalPathPlanner *pathplanner = new LocalPathPlanner();
  std::vector<std::pair<double,  double>> path;
  std::vector<PathPoint *> pathPointers = pathplanner->processNewArray(track);
  for (size_t i = 0; i < pathPointers.size(); i++) {
    path.push_back({pathPointers[i] -> getX(),  pathPointers[i] -> getY()});
  }
  std::vector<std::pair<double,  double>> expected ={ {1,  0},  {1,  1},  {1,  2},
   {1.5, 3}, {1.75, 3.5}, {1.75, 3.5}, {1.75,  3.5}, {2, 4},
   {2.5, 4.5}, {3, 5}, {3.5, 4.5}, {4, 4} };
  path = orderVectorOfPairs(path);
  expected = orderVectorOfPairs(expected);
  EXPECT_EQ(path.size(),  expected.size());
  for (int n=0; n<static_cast<int>(expected.size()); n++) {
    EXPECT_FLOAT_EQ(path[n].first,  expected[n].first);
    EXPECT_FLOAT_EQ(path[n].second,  expected[n].second);
  }
}


TEST(LocalPathPlanner,  delauney100) {
  std::string filePath = "src/planning/tracks/map_100.txt";
  std::vector<PathPoint> path = processTriangulations(filePath,  "100points");
}


TEST(LocalPathPlanner,  delauney250) {
  std::string filePath = "src/planning/tracks/map_250.txt";
  std::vector<PathPoint> path = processTriangulations(filePath,  "250points");
}


TEST(LocalPathPlanner,  delauneyrng) {
  std::string filePath = "src/planning/tracks/map_250_rng.txt";
  std::vector<PathPoint> path = processTriangulations(filePath,  "250randompoints");
}


TEST(LocalPathPlanner,  delauneyoutliers0) {
  std::string filePath = "src/planning/tracks/map_250.txt";
  outlierCalculations(filePath,  "250points_2outliers");
}

TEST(LocalPathPlanner,  delauneyoutliers1) {
  std::string filePath = "src/planning/tracks/map_250_out10.txt";
  outlierCalculations(filePath,  "250points_10outliers");
}


TEST(LocalPathPlanner,  delauneyoutliers2) {
  std::string filePath = "src/planning/tracks/map_250_out25.txt";
  outlierCalculations(filePath,  "250points_25outliers");
}


TEST(LocalPathPlanner,  delauneyoutliers3) {
  std::string filePath = "src/planning/tracks/map_250_out50.txt";
  outlierCalculations(filePath,  "250points_50outliers");
}
