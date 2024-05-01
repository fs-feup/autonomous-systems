#include <time.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream> /**< To write file */
#include <memory>
#include <random>
#include <string>

#include "gtest/gtest.h"
#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class ExecTimeTestEKFTests
 * @brief Test class for performance measurements of Extended Kalman Filter
 * steps.
 */
class PerformamceTest : public ::testing::Test {
 public:
  std::unique_ptr<ExtendedKalmanFilter>
      ekf_test; /**< Unique pointer to ExtendedKalmanFilter object */
  std::unique_ptr<MotionUpdate> motion_update_test; /**< Unique pointer to MotionUpdate object */
  std::chrono::_V2::system_clock::time_point
      start_time; /**< Start time for performance measurement */

  std::chrono::_V2::system_clock::time_point
      prediction_step_time; /**< Prediction Step Execution Time for performance measurement */

  std::chrono::_V2::system_clock::time_point end_time; /**< End time for performance measurement */

  std::chrono::microseconds prediction_step_duration; /**< Duration of prediction step execution */
  std::chrono::microseconds correction_step_duration; /**< Duration of correction step execution */
  std::chrono::microseconds duration;
  std::string workload;   /**< Description of the workload */
  Eigen::Matrix2f Q_test; /**< Test Eigen Matrix for Q */
  Eigen::MatrixXf R_test; /**< Test Eigen Matrix for R */
  std::unique_ptr<MotionModel>
      motion_model_test; /**< Pointer to the MotionModel object for testing */
  int prediction_step_input_size = 0;
  int correction_step_input_size = 0;
  ObservationModel observation_model_test =
      ObservationModel(Q_test); /**< ObservationModel object for testing */
  std::string file_name;        /**< File Name for Output */

  /**
   * @brief Get current date and time as a string.
   * @return Current date and time as a string in "YYYY-MM-DD-HH:MM" format.
   */
  std::string getCurrentDateTimeAsString() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm;
    localtime_r(&now_time, &now_tm);

    std::stringstream ss;
    ss << std::put_time(&now_tm, "%Y-%m-%d-%H:%M");
    return ss.str();
  }

  /**
   * @brief Writes performance data to a CSV file.
   */
  void print_to_file() {
    bool fileExists = std::filesystem::exists("../../performance/exec_time/loc_map/" + file_name);
    std::ofstream file("../../performance/exec_time/loc_map/" + file_name,
                       std::ios::app); /**< Output file for performance data (append) */

    // Check if the file exists
    if (!fileExists) {
      // If the file doesn't exist, add header
      file << "Number of Cones in the Map, Prediction Step Execution Time, Number of Cones coming "
              "from perception, Correction Step Execution Time (ms), Total time (ms)\n";
    }

    // Convert the duration from microseconds to milliseconds
    double prediction_step_duration_milliseconds =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(prediction_step_duration)
                .count()) /
        1000.0;

    double correction_step_duration_milliseconds =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(correction_step_duration)
                .count()) /
        1000.0;

    double milliseconds =
        prediction_step_duration_milliseconds + correction_step_duration_milliseconds;

    printf("The Test Duration Was: %f ms\n", milliseconds);

    // Append performance data

    file << prediction_step_input_size << "," << prediction_step_duration_milliseconds << ","
         << correction_step_input_size << "," << correction_step_duration_milliseconds << ","
         << milliseconds << std::endl;

    file.close();
  }

  /**
   * @brief Fills the state vector with values from a file
   * @param size Number of state vector elements to fill.
   */
  void fill_X(int size) {
    ekf_test = std::make_unique<ExtendedKalmanFilter>(*motion_model_test, observation_model_test);
    std::ifstream file("../../src/loc_map/test/data/map_test.txt");

    if (!file.is_open()) {
      return;
    }

    int i = 0;
    bool is_blue = false;
    double value1;
    double value2;

    ekf_test->init_X_size(size * 2 + 3);
    ekf_test->set_P(size * 2 + 3);

    while ((file >> value1 >> value2) && i < (size * 2 + 2)) {
      ekf_test->set_X_y(i, value1);
      i++;
      ekf_test->set_X_y(i, value2);
      i++;

      if (i <= 12) {
        ekf_test->push_to_colors(colors::Color::blue);
        continue;
      }

      else if (i >= 12 && i <= 22) {
        ekf_test->push_to_colors(colors::Color::yellow);
        continue;
      }

      is_blue ? ekf_test->push_to_colors(colors::Color::blue)
              : ekf_test->push_to_colors(colors::Color::yellow);
      is_blue = !is_blue;
    }

    file.close();
  }

  void SetUp() override { /**< Set up test environment */
    file_name = "loc_map_" + getCurrentDateTimeAsString() + ".csv";
    start_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(start_time - start_time);
    prediction_step_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(start_time - start_time);
    correction_step_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(start_time - start_time);
    motion_update_test = std::make_unique<MotionUpdate>();
    motion_update_test->translational_velocity = 1.58113883;
    motion_update_test->translational_velocity_x = 1.5;
    motion_update_test->translational_velocity_y = 0.5;
    motion_update_test->rotational_velocity = 6.0;
    motion_update_test->steering_angle = 2.0;
    motion_update_test->last_update = std::chrono::high_resolution_clock::now();

    Q_test = Eigen::Matrix2f::Zero();
    Q_test(0, 0) = 0.3;
    Q_test(1, 1) = 0.3;
    R_test = Eigen::Matrix3f::Zero();
    R_test(0, 0) = 0.8;
    R_test(1, 1) = 0.8;
    R_test(2, 2) = 0.8;
    motion_model_test = std::make_unique<NormalVelocityModel>(R_test);
    observation_model_test = ObservationModel(Q_test);
    ekf_test = std::make_unique<ExtendedKalmanFilter>(*motion_model_test, observation_model_test);
  }

  /**
   * @brief Runes the Execution 10 times and outputs the average execution time
   *
   * @param coneMap Perception's output map
   */
  void runExecution(ConeMap coneMap) {
    prediction_step_duration = std::chrono::microseconds::zero();
    correction_step_duration = std::chrono::microseconds::zero();

    for (int i = 0; i < 10; i++) {
      auto ekf_for_predict = *ekf_test;
      auto ekf_for_correction = *ekf_test;

      start_time = std::chrono::high_resolution_clock::now();
      ekf_for_predict.prediction_step(*motion_update_test);
      prediction_step_time = std::chrono::high_resolution_clock::now();
      ekf_for_correction.correction_step(coneMap);
      end_time = std::chrono::high_resolution_clock::now();

      prediction_step_duration +=
          std::chrono::duration_cast<std::chrono::microseconds>(prediction_step_time - start_time);
      correction_step_duration +=
          std::chrono::duration_cast<std::chrono::microseconds>(end_time - prediction_step_time);
    }

    prediction_step_duration = prediction_step_duration / 10;
    correction_step_duration = correction_step_duration / 10;
  }

  /**
   * @brief Read and pares a file containing containing perception's output
   *
   * @param n_cones Number of cones to output
   * @return ConeMap Data Structure representing the cone map coming from perception
   */
  ConeMap createConeMap(int n_cones) {
    ConeMap coneMap;

    correction_step_input_size = 0;

    std::ifstream file("../../src/loc_map/test/data/perception_output.csv");
    std::string line;

    if (!file.is_open()) {
      return coneMap;
    }

    while (std::getline(file, line) && correction_step_input_size < n_cones) {
      std::istringstream iss(line);
      std::string color;
      std::vector<std::string> tokens;
      std::string token;

      while (std::getline(iss, token, ',')) {
        tokens.push_back(token);
      }

      Position conePosition(std::stof(tokens[0]), std::stof(tokens[1]));

      coneMap.map[conePosition] = tokens[2] == "blue" ? colors::blue : colors::yellow;
      correction_step_input_size++;
    }

    file.close();

    return coneMap;
  }
};

/**
 * @brief EKF Performance Test
 *
 */
TEST_F(PerformamceTest, TEST_EKF_PERFORMANCE) {
  for (int i = 0; i <= 10; i++) {
    ConeMap coneMap = createConeMap(i);
    for (int j = 10; j <= 200; j += 10) {
      prediction_step_input_size = j;
      fill_X(j);
      runExecution(coneMap);
      print_to_file();
    }
  }
}