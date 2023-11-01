#include <fstream>  //to write file

#include "gtest/gtest.h"
#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"
// microsegundos - /100

class ExecTimeTestEKFTests : public ::testing::Test {
 protected:
  ExtendedKalmanFilter *ekf_test;
  MotionUpdate *motion_update_test;
  std::chrono::_V2::system_clock::time_point start_time;
  std::chrono::microseconds duration;
  std::chrono::_V2::system_clock::time_point end_time;
  std::string workload;
  Eigen::Matrix2f Q_test;
  Eigen::MatrixXf R_test;
  MotionModel *motion_model_test;
  ObservationModel observation_model_test = ObservationModel(Q_test);

  void print_to_file() {
    std::ofstream file("../../performance/exec_time/loc_map.csv", std::ios::app);  // append

    // Convert the duration from microseconds to milliseconds
    double milliseconds =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(duration).count()) /
        1000.0;

    file << "LOC_MAP, " << workload << ", " << std::fixed << milliseconds << " ms\n";
    file.close();
  }

  void fill_X(int size) {
    for (int i = 3; i <= size; i++) {
      double randomX = (static_cast<double>(rand() / RAND_MAX)) * 50.0;
      double randomY = (static_cast<double>(rand() / RAND_MAX)) * 50.0;

      // Call set_X_y for X and Y values
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

  void SetUp() override {  // TODO(PedroRomao3) //SetUpTestSuite

    start_time = std::chrono::high_resolution_clock::now();
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    motion_update_test = new MotionUpdate();
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
  void TearDown() override {
    delete ekf_test;
    delete motion_update_test;
  }
};

TEST_F(ExecTimeTestEKFTests, TEST_EKF_PRED_10) {
  // create motion update and state with adequate workload

  motion_update_test->translational_velocity = 1.58113883;
  motion_update_test->translational_velocity_x = 1.5;
  motion_update_test->translational_velocity_y = 0.5;
  motion_update_test->rotational_velocity = 6.0;
  motion_update_test->steering_angle = 2.0;
  motion_update_test->last_update = std::chrono::high_resolution_clock::now();
  ekf_test->init_X_size(23);
  ekf_test->set_P(23);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  // necessary?
  ekf_test->set_X_y(3, -1.637208342552185);
  ekf_test->set_X_y(4, 14.400202751159668);
  ekf_test->push_to_colors(colors::Color::blue);
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
  //\necessary?

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->prediction_step(*motion_update_test);
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF pred step, 10";
  print_to_file();

}
TEST_F(ExecTimeTestEKFTests, TEST_EKF_PRED_50) {
  // create motion update and state with adequate workload

  motion_update_test->translational_velocity = 1.58113883;
  motion_update_test->translational_velocity_x = 1.5;
  motion_update_test->translational_velocity_y = 0.5;
  motion_update_test->rotational_velocity = 6.0;
  motion_update_test->steering_angle = 2.0;
  motion_update_test->last_update = std::chrono::high_resolution_clock::now();
  ekf_test->init_X_size(103);
  ekf_test->set_P(103);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);

  fill_X(102);

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->prediction_step(*motion_update_test);

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF pred step, 50";
  print_to_file();

}
TEST_F(ExecTimeTestEKFTests, TEST_EKF_PRED_100) {

  ekf_test->init_X_size(203);
  ekf_test->set_P(203);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);

  fill_X(202);

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->prediction_step(*motion_update_test);

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF pred step, 100";
  print_to_file();

}

TEST_F(ExecTimeTestEKFTests, TEST_EKF_CORR_10) {
  ConeMap coneMap;
  for (int i = 0; i < 10; i++) {
    Position conePosition(i * 2.0, i * 2.0);

    // Add the cone to the map
    coneMap.map[conePosition] = colors::blue;
  }
  coneMap.last_update = std::chrono::high_resolution_clock::now();

  ekf_test->init_X_size(23);
  ekf_test->set_P(23);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  ekf_test->set_X_y(3, -1.637208342552185);
  ekf_test->set_X_y(4, 14.400202751159668);
  ekf_test->push_to_colors(colors::Color::blue);
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

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->correction_step(coneMap);

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF Correction Step, 10 and 10 From \"perception\"";
  print_to_file();

}

TEST_F(ExecTimeTestEKFTests, TEST_EKF_CORR_50) {
  // create cone map , maybe state
  ConeMap coneMap;
  for (int i = 0; i < 10; i++) {
    Position conePosition(i * 2.0, i * 2.0);

    // Add the cone to the map
    coneMap.map[conePosition] = colors::blue;
  }
  coneMap.last_update = std::chrono::high_resolution_clock::now();

  ekf_test->init_X_size(103);
  ekf_test->set_P(103);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  // necessary?
  fill_X(102);

  start_time = std::chrono::high_resolution_clock::now();

  ekf_test->correction_step(coneMap);

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF Correction Step, 50 and 10 From \"perception\"";
  print_to_file();

}

TEST_F(ExecTimeTestEKFTests, TEST_EKF_CORR_100) {
  // create cone map , maybe state
  ConeMap coneMap;
  for (int i = 0; i < 10; i++) {
    Position conePosition(i * 2.0, i * 2.0);

    // Add the cone to the map
    coneMap.map[conePosition] = colors::blue;
  }
  coneMap.last_update = std::chrono::high_resolution_clock::now();
  ekf_test->init_X_size(203);
  ekf_test->set_P(203);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  fill_X(202);
  for (int i = 0; i < 100; i++) {
    start_time = std::chrono::high_resolution_clock::now();

    ekf_test->correction_step(coneMap);

    end_time = std::chrono::high_resolution_clock::now();
    duration += std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
                std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time));
  }

  duration = duration / 100;
  workload = "EKF Correction Step, 100 and 10 From \"perception\"";
  print_to_file();

}