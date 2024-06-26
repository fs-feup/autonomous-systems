#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "test_utils/utils.hpp"

/**
 * @brief Iterates the Outliers algorithm repeatedly and measures the average time
 */
void iterate_outliers(const std::string &filename, int num_outliers = 0) {
  auto track = track_from_file(filename);
  auto outliers = Outliers();

  // Remove outliers no_iters times to get average
  int no_iters = 100;
  double total_time = 0;

  std::ofstream measures_path = openWriteFile(
      "performance/exec_time/planning/planning_" + get_current_date_time_as_string() + ".csv",
      "Number of Left Cones,Number of Right Cones,Number of "
      "Outliers,Outliers Removal Execution "
      "Time,Triangulations Execution Time,Smoothing Execution Time");

  for (int i = 0; i < no_iters; i++) {
    track = track_from_file(filename);
    auto t0 = std::chrono::high_resolution_clock::now();
    track = outliers.approximate_cones_with_spline(track);
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed_time_iter_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    total_time += elapsed_time_iter_ms;
  }

  measures_path << track.first.size() << "," << track.second.size() << "," << num_outliers << ","
                << total_time / no_iters << ",";
  measures_path.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Outliers removed in average %f ms.",
              (total_time / no_iters));
}

/**
 * @brief Iterates the Path Calculation algorithm repeatedly and measures the average time
 */
std::vector<PathPoint> iterate_triangulations(const std::string &filename) {
  auto track = track_from_file(filename);
  auto path_calculation = PathCalculation();

  std::vector<PathPoint> path;
  std::ofstream measures_path = openWriteFile("performance/exec_time/planning/planning_" +
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

  measures_path << total_time / no_iters << ",";
  measures_path.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Average Delaunay Triangulations processed in %f ms.",
              (total_time / no_iters));

  return path;
}

/**
 * @brief Iterates the Smoothing algorithm repeatedly and measures the average time
 */
void iterate_smoothing(std::vector<PathPoint> &path) {
  auto path_smoothing = PathSmoothing();

  std::ofstream measures_path = openWriteFile("performance/exec_time/planning/planning_" +
                                              get_current_date_time_as_string() + ".csv");
  double total_time = 0;
  int no_iters = 100;

  // No_iters repetitions to get average
  for (int i = 0; i < no_iters; i++) {
    float orientation = static_cast<float>(
        atan2(path[1].position.y - path[0].position.y, path[1].position.x - path[0].position.x));

    auto t0 = std::chrono::high_resolution_clock::now();

    std::vector<PathPoint> smoothed_path =
        path_smoothing.smooth_path(path, Pose(path[0].position, orientation));

    auto t1 = std::chrono::high_resolution_clock::now();

    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    total_time += elapsed_time_ms;
  }

  measures_path << total_time / no_iters << "\n";
  measures_path.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Average Smoothing processed in %f ms.",
              (total_time / no_iters));
}

/**
 * @brief Execution Time Test
 */
TEST(Planning, planning_exec_time) {
  std::string directory_path = "../../src/planning/test/maps/";
  int size;
  int n_outliers;
  for (const auto &entry : fs::directory_iterator(directory_path)) {
    if (fs::is_regular_file(entry.path())) {
      std::string filename = entry.path().filename().string();
      if (filename.find("map_") != std::string::npos) {
        extract_info(filename, size, n_outliers);
        std::string filePath = "src/planning/test/maps/" + filename;
        iterate_outliers(filePath, n_outliers);
        std::vector<PathPoint> path = iterate_triangulations(filePath);
        iterate_smoothing(path);
      }
    }
  }
}