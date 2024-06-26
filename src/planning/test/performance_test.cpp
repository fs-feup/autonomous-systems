#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "test_utils/utils.hpp"

void iterate_coloring(std::string filename) {
  std::vector<common_lib::structures::Cone> cones =
      cone_vector_from_file(filename);
  auto cone_coloring = ConeColoring();

  std::ofstream measuresPath = openWriteFile("performance/exec_time/planning/planning_" +
                                             get_current_date_time_as_string() + ".csv");

  int no_iters = 100;
  double total_time = 0;

  // No_iters repetitions to get average
  for (int i = 0; i < no_iters; i++) {
    float orientation = atan2(cones[1].position.y - cones[0].position.y, cones[1].position.x - cones[0].position.x);

    auto t0 = std::chrono::high_resolution_clock::now();

    auto colored_cones = cone_coloring.color_cones(cones, Pose(cones[0].position.x, cones[0].position.y - 1.5, orientation));

    auto t1 = std::chrono::high_resolution_clock::now();

    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    total_time += elapsed_time_ms;
  }

  measuresPath << total_time / no_iters << ",";
  measuresPath.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cones colored in average %f ms.",
              (total_time / no_iters));
}

/**
 * @brief Iterates the Outliers algorithm repeatedly and measures the average time
 */
void iterate_outliers(std::string filename, int num_outliers = 0) {
    auto track = track_from_file(filename);
    auto outliers = Outliers();

  // Remove outliers no_iters times to get average
  int no_iters = 100;
  double total_time = 0;

  std::ofstream measuresPath = openWriteFile(
      "performance/exec_time/planning/planning_" + get_current_date_time_as_string() + ".csv",
      "Number of Left Cones,Number of Right Cones,Number of "
      "Outliers,Outliers Removal Execution "
      "Time,Cone Coloring Execution Time,Triangulations Execution Time,Smoothing Execution Time");

  for (int i = 0; i < no_iters; i++) {
    track = track_from_file(filename);
    auto t0 = std::chrono::high_resolution_clock::now();
    track = outliers.approximate_cones_with_spline(track);
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed_time_iter_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    total_time += elapsed_time_iter_ms;
  }

  measuresPath << track.first.size() << "," << track.second.size() << ","
               << num_outliers << "," << total_time / no_iters << ",";
  measuresPath.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Outliers removed in average %f ms.",
              (total_time / no_iters));
}

/**
 * @brief Iterates the Path Calculation algorithm repeatedly and measures the average time
 */
std::vector<PathPoint> iterate_triangulations(std::string filename) {
  auto track = track_from_file(filename);
  auto path_calculation = PathCalculation();

  std::vector<PathPoint> path;
  std::ofstream measuresPath = openWriteFile("performance/exec_time/planning/planning_" +
                                             get_current_date_time_as_string() + ".csv");
  double total_time = 0;
  int no_iters = 100;

  // No_iters repetitions to get average
  for (int i = 0; i < no_iters; i++) {
    auto t0 = std::chrono::high_resolution_clock::now();

    path = path_calculation.process_delaunay_triangulations(track);

    auto t1 = std::chrono::high_resolution_clock::now();

    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    total_time += elapsed_time_ms;
  }

  measuresPath << total_time / no_iters << ",";
  measuresPath.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Average Delaunay Triangulations processed in %f ms.",
              (total_time / no_iters));

  return path;
}

/**
 * @brief Iterates the Smoothing algorithm repeatedly and measures the average time
 */
void iterate_smoothing(std::vector<PathPoint> &path) {
  auto path_smoothing = PathSmoothing();

  std::ofstream measuresPath = openWriteFile("performance/exec_time/planning/planning_" +
                                             get_current_date_time_as_string() + ".csv");
  double total_time = 0;
  int no_iters = 100;

  // No_iters repetitions to get average
  for (int i = 0; i < no_iters; i++) {
    float orientation = atan2(path[1].position.y - path[0].position.y, path[1].position.x - path[0].position.x);

    auto t0 = std::chrono::high_resolution_clock::now();

    std::vector<PathPoint> smoothed_path = path_smoothing.smooth_path(path, Pose(path[0].position, orientation));

    auto t1 = std::chrono::high_resolution_clock::now();

    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    total_time += elapsed_time_ms;
  }

  measuresPath << total_time / no_iters << "\n";
  measuresPath.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Average Smoothing processed in %f ms.",
              (total_time / no_iters));

}

/**
 * @brief Execution Time Test
 */
TEST(Planning, planning_exec_time) {
  std::string directory_path = "src/planning/test/maps/";
  int size;
  int n_outliers;
  for (const auto &entry : fs::directory_iterator(directory_path)) {
    if (fs::is_regular_file(entry.path())) {
      std::string filename = entry.path().filename().string();
      if (filename.find("map_") != std::string::npos) {
        extractInfo(filename, size, n_outliers);
        std::string filePath = "src/planning/test/maps/" + filename;
        iterate_outliers(filePath, n_outliers);
        iterate_coloring(filePath);
        std::vector<PathPoint> path = iterate_triangulations(filePath);
        iterate_smoothing(path);
      }
    }
  }
}