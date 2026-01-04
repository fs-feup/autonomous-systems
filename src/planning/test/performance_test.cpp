#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "test_utils/utils.hpp"


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

    auto t0 = std::chrono::high_resolution_clock::now();

    std::vector<PathPoint> smoothed_path =
        path_smoothing.smooth_path(path);

    auto t1 = std::chrono::high_resolution_clock::now();

    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    total_time += elapsed_time_ms;
  }

  measures_path << total_time / no_iters << "\n";
  measures_path.close();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Average Smoothing processed in %f ms.",
              (total_time / no_iters));
}
