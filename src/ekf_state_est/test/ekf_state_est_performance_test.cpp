#include <time.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream> /**< To write file */
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "common_lib/structures/cone.hpp"
#include "gtest/gtest.h"
#include "kalman_filter/ekf.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class ExecTimeTestEKFTests
 * @brief Test class for performance measurements of Extended Kalman Filter
 * steps.
 */
class PerformamceTest : public ::testing::Test {
public:
  std::shared_ptr<ExtendedKalmanFilter>
      ekf_test; /**< shared pointer to ExtendedKalmanFilter object */
  std::shared_ptr<MotionUpdate> motion_update_test; /**< shared pointer to MotionUpdate object */
  std::chrono::_V2::system_clock::time_point
      start_time; /**< Start time for performance measurement */

  std::chrono::_V2::system_clock::time_point
      prediction_step_time; /**< Prediction Step Execution Time for performance
                               measurement */

  std::chrono::_V2::system_clock::time_point end_time; /**< End time for performance measurement */

  std::chrono::microseconds prediction_step_duration; /**< Duration of prediction step execution */
  std::chrono::microseconds correction_step_duration; /**< Duration of correction step execution */
  std::chrono::microseconds duration;
  std::string workload;                       /**< Description of the workload */
  Eigen::Matrix2f Q_test;                     /**< Test Eigen Matrix for Q */
  Eigen::MatrixXf R_test;                     /**< Test Eigen Matrix for R */
  std::shared_ptr<MotionModel> motion_model_; /**< Pointer to the MotionModel object for testing */
  std::shared_ptr<DataAssociationModel> data_association_model_;
  int prediction_step_input_size = 0;
  int correction_step_input_size = 0;
  std::shared_ptr<ObservationModel> observation_model_; /**< ObservationModel object for testing */
  std::string file_name;                                /**< File Name for Output */

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
   * @brief Writes performance data to a CSV file.
   */
  void print_to_file() {
    bool file_exists =
        std::filesystem::exists("../../performance/exec_time/ekf_state_est/" + file_name);
    std::ofstream file("../../performance/exec_time/ekf_state_est/" + file_name,
                       std::ios::app); /**< Output file for performance data (append) */

    // Check if the file exists
    if (!file_exists) {
      // If the file doesn't exist, add header
      file << "Number of Cones in the Map, Prediction Step Execution Time, "
              "Number of Cones coming "
              "from perception, Correction Step Execution Time (ms), Total "
              "time (ms)\n";
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
    ekf_test = std::make_shared<ExtendedKalmanFilter>(*motion_model_, *observation_model_,
                                                      *data_association_model_);
    std::ifstream file("../../src/ekf_state_est/test/data/map_test.txt");

    if (!file.is_open()) {
      return;
    }

    int i = 0;
    double value1;
    double value2;

    ekf_test->init_X_size(size * 2 + 3);
    ekf_test->set_P(size * 2 + 3);

    while ((file >> value1 >> value2) && i < (size * 2 + 2)) {
      ekf_test->set_X_y(i, value1);
      i++;
      ekf_test->set_X_y(i, value2);
      i++;
    }

    file.close();
  }

  void SetUp() override { /**< Set up test environment */
    file_name = "ekf_state_est_" + get_current_date_time_as_string() + ".csv";
    start_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(start_time - start_time);
    prediction_step_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(start_time - start_time);
    correction_step_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(start_time - start_time);
    motion_update_test = std::make_shared<MotionUpdate>();
    motion_update_test->translational_velocity = 1.58113883;
    motion_update_test->translational_velocity_x = 1.5;
    motion_update_test->translational_velocity_y = 0.5;
    motion_update_test->rotational_velocity = 6.0;
    motion_update_test->steering_angle = 2.0;
    motion_update_test->last_update = rclcpp::Clock().now();

    Q_test = Eigen::Matrix2f::Zero();
    Q_test(0, 0) = 0.3;
    Q_test(1, 1) = 0.3;
    R_test = Eigen::MatrixXf::Zero(5, 5);
    R_test(0, 0) = 0.8;
    R_test(1, 1) = 0.8;
    R_test(2, 2) = 0.8;
    R_test(3, 3) = 0.8;
    R_test(4, 4) = 0.8;
    motion_model_ = std::make_shared<NormalVelocityModel>(R_test);
    data_association_model_ = std::make_shared<SimpleMaximumLikelihood>(71.0);
    observation_model_ = std::make_shared<ObservationModel>(Q_test);
    ekf_test = std::make_shared<ExtendedKalmanFilter>(*motion_model_, *observation_model_,
                                                      *data_association_model_);
  }

  /**
   * @brief Runes the Execution 10 times and outputs the average execution time
   *
   * @param cone_map Perception's output map
   */
  void run_execution(const std::vector<common_lib::structures::Cone>& cone_map) {
    prediction_step_duration = std::chrono::microseconds::zero();
    correction_step_duration = std::chrono::microseconds::zero();

    for (int i = 0; i < 10; i++) {
      auto ekf_for_predict = *ekf_test;
      auto ekf_for_correction = *ekf_test;

      start_time = std::chrono::high_resolution_clock::now();
      ekf_for_predict.prediction_step(*motion_update_test);
      prediction_step_time = std::chrono::high_resolution_clock::now();
      ekf_for_correction.correction_step(cone_map);
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
   * @return std::vector<common_lib::structures::Cone> Data Structure representing the cone map
   * coming from perception
   */
  std::vector<common_lib::structures::Cone> create_cone_map(int n_cones) {
    std::vector<common_lib::structures::Cone> cone_map;

    correction_step_input_size = 0;

    std::ifstream file("../../src/ekf_state_est/test/data/perception_output.csv");
    std::string line;

    if (!file.is_open()) {
      return cone_map;
    }

    while (std::getline(file, line) && correction_step_input_size < n_cones) {
      std::istringstream iss(line);
      std::string color;
      std::vector<std::string> tokens;
      std::string token;

      while (std::getline(iss, token, ',')) {
        tokens.push_back(token);
      }

      common_lib::structures::Cone cone(std::stof(tokens[0]), std::stof(tokens[1]), tokens[2], 1.0);

      cone_map.push_back(cone);
      correction_step_input_size++;
    }

    file.close();

    return cone_map;
  }
};

/**
 * @brief EKF Performance Test
 *
 */
TEST_F(PerformamceTest, TEST_EKF_PERFORMANCE) {
  std::vector<int> cones_detected_cases = {1, 2, 5, 10};
  std::vector<int> map_size_cases = {10, 20, 50, 100, 200};
  for (auto i : cones_detected_cases) {
    std::vector<common_lib::structures::Cone> cone_map = create_cone_map(i);
    for (auto j : map_size_cases) {
      prediction_step_input_size = j;
      fill_X(j);
      run_execution(cone_map);
      print_to_file();
    }
  }
}