#include <fstream> /**< To write file */
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "speed_est/se_node.hpp"

/**
 * @class ExecTimeTestEKFTests
 * @brief Test class for performance measurements of Extended Kalman Filter
 * steps.
 */
class ExecTimeTestEKFTests : public ::testing::Test {
 protected:
  ExtendedKalmanFilter *ekf_test;   /**< Pointer to the ExtendedKalmanFilter object for testing */
  MotionUpdate *motion_update_test; /**< Pointer to the MotionUpdate object for testing */
  std::chrono::_V2::system_clock::time_point
      start_time;                     /**< Start time for performance measurement */
  std::chrono::microseconds duration; /**< Duration of execution */
  std::chrono::_V2::system_clock::time_point end_time; /**< End time for performance measurement */
  std::string workload;                                /**< Description of the workload */
  Eigen::Matrix2f Q_test;                              /**< Test Eigen Matrix for Q */
  Eigen::MatrixXf R_test;                              /**< Test Eigen Matrix for R */
  MotionModel *motion_model_test; /**< Pointer to the MotionModel object for testing */
  ObservationModel observation_model_test =
      ObservationModel(Q_test); /**< ObservationModel object for testing */

  /**
   * @brief Writes the covariance matrix P to a CSV file.
   */
  void print_P() {
    // Get covariance matrix
    Eigen::MatrixXf P = ekf_test->get_covariance();

    // Open file
    std::ofstream file("../../src/performance/exec_time/loc_map_P.csv", std::ios::app);

    // Write P to file
    for (int i = 0; i < P.rows(); i++) {
      for (int j = 0; j < P.cols(); j++) {
        file << P(i, j);
        if (j != P.cols() - 1) {
          file << ",";
        }
      }
      file << "\n";
    }
    file << "\n\n";
    // Close file
    file.close();
  }

  /**
   * @brief Writes performance data to a CSV file.
   */
  void print_to_file() {
    std::ofstream file("../../src/performance/exec_time/loc_map.csv",
                       std::ios::app); /**< Output file for performance data (append) */

    // Convert the duration from microseconds to milliseconds
    double milliseconds =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(duration).count()) /
        1000.0;
    printf("The Test Duration Was: %f ms\n", milliseconds);
    // print current working directory
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
      printf("Current working dir: %s\n", cwd);
    } else {
      perror("getcwd() error");
    }
    // end print current working directory
    file << "LOC_MAP, " << workload << ", " << std::fixed << milliseconds << " ms\n";
    file.close();
  }

  /**
   * @brief Fills the state vector with random values.
   * @param size ~Number of state vector elements to fill.
   */
  void fill_X(int size) {
    ekf_test->set_X_y(0, -15.0);  // TODO(pedroromao3): repeated initialization fix
    ekf_test->set_X_y(1, 0.0);
    ekf_test->set_X_y(2, 0.0);
    ekf_test->set_X_y(3, 0.0);
    ekf_test->set_X_y(4, 0.0);
    ekf_test->set_X_y(5, -2.216218948364258);
    ekf_test->set_X_y(6, 11.487205505371094);
    ekf_test->push_to_colors(colors::Color::blue);
    ekf_test->set_X_y(7, -3.867227792739868);
    ekf_test->set_X_y(8, 9.018211364746094);
    ekf_test->push_to_colors(colors::Color::blue);

    ekf_test->set_X_y(9, -6.336233615875244);
    ekf_test->set_X_y(10, 7.367220401763916);
    ekf_test->push_to_colors(colors::Color::blue);

    ekf_test->set_X_y(11, -9.250235557556152);
    ekf_test->set_X_y(12, 6.788230895996094);
    ekf_test->push_to_colors(colors::Color::blue);

    ekf_test->set_X_y(13, 16.861791610717773);
    ekf_test->set_X_y(14, 14.40013599395752);
    ekf_test->push_to_colors(colors::Color::yellow);

    ekf_test->set_X_y(15, 16.28278160095215);
    ekf_test->set_X_y(16, 11.487138748168945);
    ekf_test->push_to_colors(colors::Color::yellow);
    ekf_test->set_X_y(17, 14.6317720413208);
    ekf_test->set_X_y(18, 9.018143653869629);
    ekf_test->push_to_colors(colors::Color::yellow);
    ekf_test->set_X_y(19, 12.162766456604004);
    ekf_test->set_X_y(20, 7.367153644561768);
    ekf_test->push_to_colors(colors::Color::yellow);
    ekf_test->set_X_y(21, 9.249764442443848);
    ekf_test->set_X_y(22, 6.788164138793945);
    ekf_test->push_to_colors(colors::Color::yellow);
    for (int i = 23; i <= size; i++) {
      double randomX = (static_cast<double>(rand() / RAND_MAX)) * 50.0;
      double randomY = (static_cast<double>(rand() / RAND_MAX)) * 50.0;

      // Call set_X_y for X and Y values, sets the value if X at y
      ekf_test->set_X_y(i, randomX);
      ekf_test->set_X_y(i + 1, randomY);

      // Call push_to_colors with the current color
      if (i % 2 == 0) {
        ekf_test->push_to_colors(colors::Color::blue);
      } else {
        ekf_test->push_to_colors(colors::Color::yellow);
      }

      // Increment the index by 2
      i++;
    }
  }

  void SetUp() override { /**< Set up test environment */
    start_time = std::chrono::high_resolution_clock::now();
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    motion_update_test = new MotionUpdate();
    motion_update_test->translational_velocity = 1.58113883;
    motion_update_test->translational_velocity_x = 1.5;
    motion_update_test->translational_velocity_y = 0.5;
    motion_update_test->rotational_velocity = 6.0;
    motion_update_test->steering_angle = 2.0;
    motion_update_test->last_update = std::chrono::high_resolution_clock::now();

    Q_test = Eigen::Matrix2f::Zero();
    Q_test(0, 0) = 0.3;
    Q_test(1, 1) = 0.3;
    R_test = Eigen::MatrixXf::Zero(5, 5);
    R_test(0, 0) = 0.8;
    R_test(1, 1) = 0.8;
    R_test(2, 2) = 0.8;
    R_test(3, 3) = 0.8;
    R_test(4, 4) = 0.8;
    motion_model_test = new NormalVelocityModel(R_test);
    observation_model_test = ObservationModel(Q_test);
    ekf_test = new ExtendedKalmanFilter(*motion_model_test, observation_model_test);
  }

  void TearDown() override {
    delete ekf_test;
    delete motion_update_test;
  }
};

/**
 * @brief Test case for the Extended Kalman Filter prediction step with a
 * workload of 10.
 */
TEST_F(ExecTimeTestEKFTests, TEST_EKF_PRED_10) {
  // fill state vector
  ekf_test->init_X_size(23);
  ekf_test->set_P(23);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  ekf_test->set_X_y(3, 0);
  ekf_test->set_X_y(4, 0);
  ekf_test->set_X_y(5, -2.216218948364258);
  ekf_test->set_X_y(6, 11.487205505371094);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(7, -3.867227792739868);
  ekf_test->set_X_y(8, 9.018211364746094);
  ekf_test->push_to_colors(colors::Color::blue);

  ekf_test->set_X_y(9, -6.336233615875244);
  ekf_test->set_X_y(10, 7.367220401763916);
  ekf_test->push_to_colors(colors::Color::blue);

  ekf_test->set_X_y(11, -9.250235557556152);
  ekf_test->set_X_y(12, 6.788230895996094);
  ekf_test->push_to_colors(colors::Color::blue);

  ekf_test->set_X_y(13, 16.861791610717773);
  ekf_test->set_X_y(14, 14.40013599395752);
  ekf_test->push_to_colors(colors::Color::yellow);

  ekf_test->set_X_y(15, 16.28278160095215);
  ekf_test->set_X_y(16, 11.487138748168945);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(17, 14.6317720413208);
  ekf_test->set_X_y(18, 9.018143653869629);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(19, 12.162766456604004);
  ekf_test->set_X_y(20, 7.367153644561768);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(21, 9.249764442443848);
  ekf_test->set_X_y(22, 6.788164138793945);
  ekf_test->push_to_colors(colors::Color::yellow);

  // run the prediction step 10 times and average the duration
  for (int i = 0; i < 10; i++) {
    start_time = std::chrono::high_resolution_clock::now();

    ekf_test->prediction_step(*motion_update_test);

    end_time = std::chrono::high_resolution_clock::now();
    duration += std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
  }

  duration = duration / 10;

  workload = "EKF pred step, 10";
  print_to_file();
}

/**
 * @brief Test case for the Extended Kalman Filter prediction step with a
 * workload of 50.
 */
TEST_F(ExecTimeTestEKFTests, TEST_EKF_PRED_50) {
  ekf_test->init_X_size(103);
  ekf_test->set_P(103);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  ekf_test->set_X_y(4, 0.0);
  ekf_test->set_X_y(5, 0.0);

  fill_X(102);

  for (int i = 0; i < 10; i++) {
    start_time = std::chrono::high_resolution_clock::now();

    ekf_test->prediction_step(*motion_update_test);

    end_time = std::chrono::high_resolution_clock::now();
    duration += std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
  }

  duration = duration / 10;

  workload = "EKF pred step, 50";
  print_to_file();
}

/**
 * @brief Test case for the Extended Kalman Filter prediction step with a
 * workload of 100.
 */
TEST_F(ExecTimeTestEKFTests, TEST_EKF_PRED_100) {
  ekf_test->init_X_size(203);
  ekf_test->set_P(203);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);

  fill_X(202);

  for (int i = 0; i < 10; i++) {
    start_time = std::chrono::high_resolution_clock::now();

    ekf_test->prediction_step(*motion_update_test);

    end_time = std::chrono::high_resolution_clock::now();
    duration += std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
  }

  duration = duration / 10;

  workload = "EKF pred step, 100";
  print_to_file();
}

/**
 * @brief Test case for the Extended Kalman Filter correction step with a
 * workload of 10.
 */
TEST_F(ExecTimeTestEKFTests, TEST_EKF_CORR_10) {
  // create conemap with 10 cones
  ConeMap coneMap;

  Position conePosition(14, 14);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(12.8, 11.4);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(11.2, 9);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(8.7, 7.3);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(5.8, 6.7);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(31.8, 14.4);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(31.2, 11.4);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(29.6, 9);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(27.1, 7.3);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(24.2, 6.7);
  coneMap.map[conePosition] = colors::yellow;

  coneMap.last_update = std::chrono::high_resolution_clock::now();

  ekf_test->init_X_size(23);
  ekf_test->set_P(23);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  ekf_test->set_X_y(3, 0);
  ekf_test->set_X_y(4, 0);
  ekf_test->set_X_y(5, -2.216218948364258);
  ekf_test->set_X_y(6, 11.487205505371094);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(7, -3.867227792739868);
  ekf_test->set_X_y(8, 9.018211364746094);
  ekf_test->push_to_colors(colors::Color::blue);

  ekf_test->set_X_y(9, -6.336233615875244);
  ekf_test->set_X_y(10, 7.367220401763916);
  ekf_test->push_to_colors(colors::Color::blue);

  ekf_test->set_X_y(11, -9.250235557556152);
  ekf_test->set_X_y(12, 6.788230895996094);
  ekf_test->push_to_colors(colors::Color::blue);

  ekf_test->set_X_y(13, 16.861791610717773);
  ekf_test->set_X_y(14, 14.40013599395752);
  ekf_test->push_to_colors(colors::Color::yellow);

  ekf_test->set_X_y(15, 16.28278160095215);
  ekf_test->set_X_y(16, 11.487138748168945);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(17, 14.6317720413208);
  ekf_test->set_X_y(18, 9.018143653869629);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(19, 12.162766456604004);
  ekf_test->set_X_y(20, 7.367153644561768);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(21, 9.249764442443848);
  ekf_test->set_X_y(22, 6.788164138793945);
  ekf_test->push_to_colors(colors::Color::yellow);
  // run 10 times and average duration
  for (int i = 0; i < 10; i++) {
    start_time = std::chrono::high_resolution_clock::now();
    ekf_test->correction_step(coneMap);
    end_time = std::chrono::high_resolution_clock::now();
    duration += std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
  }

  duration = duration / 10;

  workload = "EKF Correction Step, 10 and 10 From \"perception\"";
  print_to_file();
}

/**
 * @brief Test case for the Extended Kalman Filter correction step with a
 * workload of 50.
 */
TEST_F(ExecTimeTestEKFTests, TEST_EKF_CORR_50) {
  ConeMap coneMap;
  Position conePosition(14, 14);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(12.8, 11.4);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(11.2, 9);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(8.7, 7.3);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(5.8, 6.7);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(31.8, 14.4);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(31.2, 11.4);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(29.6, 9);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(27.1, 7.3);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(24.2, 6.7);
  coneMap.map[conePosition] = colors::yellow;
  coneMap.last_update = std::chrono::high_resolution_clock::now();
  ekf_test->init_X_size(103);
  ekf_test->set_P(103);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  fill_X(102);
  for (int i = 0; i < 10; i++) {
    start_time = std::chrono::high_resolution_clock::now();
    ekf_test->correction_step(coneMap);
    end_time = std::chrono::high_resolution_clock::now();
    duration += std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
  }
  duration = duration / 10;
  workload = "EKF Correction Step, 50 and 10 From \"perception\"";
  print_to_file();
}

/**
 * @brief Test case for the Extended Kalman Filter correction step with a
 * workload of 100.
 */
TEST_F(ExecTimeTestEKFTests, TEST_EKF_CORR_100) {
  ConeMap coneMap;
  Position conePosition(14, 14);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(12.8, 11.4);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(11.2, 9);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(8.7, 7.3);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(5.8, 6.7);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(31.8, 14.4);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(31.2, 11.4);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(29.6, 9);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(27.1, 7.3);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(24.2, 6.7);
  coneMap.map[conePosition] = colors::yellow;
  coneMap.last_update = std::chrono::high_resolution_clock::now();
  ekf_test->init_X_size(203);
  ekf_test->set_P(203);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  fill_X(202);
  for (int i = 0; i < 10; i++) {
    start_time = std::chrono::high_resolution_clock::now();
    ekf_test->correction_step(coneMap);
    end_time = std::chrono::high_resolution_clock::now();
    duration += std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
  }
  duration = duration / 10;
  workload = "EKF Correction Step, 100 and 10 From \"perception\"";
  print_to_file();
}

/**
 * @brief Test case for the Extended Kalman Filter correction step with a
 * workload of 200.
 */
TEST_F(ExecTimeTestEKFTests, TEST_EKF_CORR_200) {
  ConeMap coneMap;
  Position conePosition(14, 14);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(12.8, 11.4);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(11.2, 9);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(8.7, 7.3);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(5.8, 6.7);
  coneMap.map[conePosition] = colors::blue;
  conePosition = Position(31.8, 14.4);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(31.2, 11.4);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(29.6, 9);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(27.1, 7.3);
  coneMap.map[conePosition] = colors::yellow;
  conePosition = Position(24.2, 6.7);
  coneMap.map[conePosition] = colors::yellow;
  coneMap.last_update = std::chrono::high_resolution_clock::now();
  ekf_test->init_X_size(403);
  ekf_test->set_P(403);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  fill_X(402);
  for (int i = 0; i < 10; i++) {
    start_time = std::chrono::high_resolution_clock::now();
    ekf_test->correction_step(coneMap);
    end_time = std::chrono::high_resolution_clock::now();
    duration += std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
  }
  duration = duration / 10;
  workload = "EKF Correction Step, 200 and 10 From \"perception\"";
  print_to_file();
}