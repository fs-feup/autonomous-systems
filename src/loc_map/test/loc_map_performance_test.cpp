#include <fstream> /**< To write file */
#include <memory>
#include <string>
#include <ctime>
#include <chrono>
#include "gtest/gtest.h"
#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include <random>
#include <time.h>

/**
 * @class ExecTimeTestEKFTests
 * @brief Test class for performance measurements of Extended Kalman Filter
 * steps.
 */
class PerformamceTest : public ::testing::Test {
 protected:
  ExtendedKalmanFilter *ekf_test;   /**< Pointer to the ExtendedKalmanFilter object for testing */
  MotionUpdate *motion_update_test; /**< Pointer to the MotionUpdate object for testing */
  std::chrono::_V2::system_clock::time_point
      start_time;                     /**< Start time for performance measurement */

  std::chrono::_V2::system_clock::time_point
      prediction_step_time;      

  std::chrono::_V2::system_clock::time_point
      end_time;               

  std::chrono::microseconds prediction_step_duration; /**< Duration of prediction step execution */
  std::chrono::microseconds correction_step_duration; /**< Duration of correction step execution */
  std::chrono::microseconds duration;
  std::string workload;                                /**< Description of the workload */
  Eigen::Matrix2f Q_test;                              /**< Test Eigen Matrix for Q */
  Eigen::MatrixXf R_test;                              /**< Test Eigen Matrix for R */
  MotionModel *motion_model_test; /**< Pointer to the MotionModel object for testing */
  int prediction_step_input_size = 0;
  int correction_step_input_size = 0;
  ObservationModel observation_model_test =
      ObservationModel(Q_test); /**< ObservationModel object for testing */
  std::string file_name;

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

  int random_number() {
    int rand_num;
    FILE *fp;
    fp = fopen("/dev/random", "r");
    fread(&rand_num, sizeof(int), 1, fp);
    fclose(fp);
    return rand_num;
  }

      /**
     * @brief Get current date and time as a string.
     * @return Current date and time as a string in "YYYY-MM-DD-HH:MM" format.
     */
    std::string getCurrentDateTimeAsString() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm* now_tm = std::localtime(&now_time);
        std::stringstream ss;
        ss << std::put_time(now_tm, "%Y-%m-%d-%H:%M");
        return ss.str();
    }

  /**
   * @brief Writes performance data to a CSV file.
   */
    void print_to_file() {

        bool fileExists = std::filesystem::exists("../../performance/exec_time/loc_map/" + file_name);
        std::ofstream file("../../performance/exec_time/loc_map/" + file_name, std::ios::app); /**< Output file for performance data (append) */

        // Check if the file exists
        if (!fileExists) {
            // If the file doesn't exist, add header
            file << "Number of Cones in the Map, Prediction Step Execution Time, Number of Cones coming from perception, Correction Step Execution Time (ms), Total time (ms)\n";
        }

        // Convert the duration from microseconds to milliseconds
        double prediction_step_duration_milliseconds =
            static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(prediction_step_duration).count()) /
            1000.0;

        double correction_step_duration_milliseconds =
            static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(correction_step_duration).count()) /
            1000.0;

        double milliseconds = prediction_step_duration_milliseconds + correction_step_duration_milliseconds;


        printf("The Test Duration Was: %f ms\n", milliseconds);
        
        // Print current working directory
        char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != NULL) {
        printf("Current working dir: %s\n", cwd);
        } else {
        perror("getcwd() error");
        }
        // End print current working directory
        
        // Append performance data

        file << prediction_step_input_size << "," << prediction_step_duration_milliseconds << "," << correction_step_input_size << "," << correction_step_duration_milliseconds << "," << milliseconds << std::endl;
        
        file.close();
    }

  /**
   * @brief Fills the state vector with random values.
   * @param size Number of state vector elements to fill.
   */
  void fill_X(int size) {
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

      time_t t;
      srand((unsigned) time(&t));

      double randomX = random_number();
      double randomY = random_number();

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
    file_name = "loc_map_" + getCurrentDateTimeAsString() + ".csv";
    start_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(start_time - start_time);
    prediction_step_duration = std::chrono::duration_cast<std::chrono::microseconds>(start_time - start_time);
    correction_step_duration = std::chrono::duration_cast<std::chrono::microseconds>(start_time - start_time);
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
    R_test = Eigen::Matrix3f::Zero();
    R_test(0, 0) = 0.8;
    R_test(1, 1) = 0.8;
    R_test(2, 2) = 0.8;
    motion_model_test = new NormalVelocityModel(R_test);
    observation_model_test = ObservationModel(Q_test);
    ekf_test = new ExtendedKalmanFilter(*motion_model_test, observation_model_test);
  }

  void runExecution(ConeMap coneMap){
    for (int i = 0; i < 10; i++) {
      start_time = std::chrono::high_resolution_clock::now();

      ekf_test->prediction_step(*motion_update_test);
      prediction_step_time = std::chrono::high_resolution_clock::now();
      ekf_test->correction_step(coneMap);
      end_time = std::chrono::high_resolution_clock::now();

      prediction_step_duration += std::chrono::duration_cast<std::chrono::microseconds>(prediction_step_time - start_time);
      correction_step_duration += std::chrono::duration_cast<std::chrono::microseconds>(end_time - prediction_step_time);
    }

    prediction_step_duration = prediction_step_duration / 10;
    correction_step_duration = correction_step_duration / 10;
  }

  ConeMap createConeMap(){
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

    return coneMap;
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
TEST_F(PerformamceTest, TEST_EKF_PRED_10) {

  // create conemap with 10 cones
  ConeMap coneMap = createConeMap();

  coneMap.last_update = std::chrono::high_resolution_clock::now();

  prediction_step_input_size = 23;
  correction_step_input_size = 10;

  ekf_test->init_X_size(23);
  ekf_test->set_P(23);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);

  fill_X(22);

  // run the prediction and correction step 10 times and average the duration
  runExecution(coneMap);

  print_to_file();
}

/**
 * @brief Test case for the Extended Kalman Filter prediction step with a
 * workload of 50
 */
TEST_F(PerformamceTest, TEST_EKF_PRED_50) {

  ConeMap coneMap = createConeMap();

  prediction_step_input_size = 53;
  correction_step_input_size = 10;

  ekf_test->init_X_size(53);
  ekf_test->set_P(53);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);

  fill_X(52);

  // run the prediction and correction step 10 times and average the duration
  runExecution(coneMap);

  print_to_file();
}



/**
 * @brief Test case for the Extended Kalman Filter prediction step with a
 * workload of .
 */
TEST_F(PerformamceTest, TEST_EKF_PRED_100) {

  ConeMap coneMap = createConeMap();

  prediction_step_input_size = 103;
  correction_step_input_size = 10;

  ekf_test->init_X_size(103);
  ekf_test->set_P(103);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);

  fill_X(102);

  // run the prediction and correction step 10 times and average the duration
  runExecution(coneMap);

  print_to_file();
}
/**
 * @brief Test case for the Extended Kalman Filter prediction step with a
 * workload of 200.
 */

TEST_F(PerformamceTest, TEST_EKF_PRED_200) {

  ConeMap coneMap = createConeMap();

  prediction_step_input_size = 203;
  correction_step_input_size = 10;


  ekf_test->init_X_size(203);
  ekf_test->set_P(203);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);

  fill_X(202);

  runExecution(coneMap);

  print_to_file();

}


TEST_F(PerformamceTest, TEST_EKF_PRED_400) {

  ConeMap coneMap = createConeMap();

  prediction_step_input_size = 403;
  correction_step_input_size = 10;

  ekf_test->init_X_size(403);
  ekf_test->set_P(403);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);

  fill_X(402);

  runExecution(coneMap);

  print_to_file();

}