#include <thread>

#include "gtest/gtest.h"
#include "kalman_filter/ekf.hpp"

/**
 * @brief Test the EKF SLAM algorithm in a linear movement
 *
 * @details This test aims to check if the EKF SLAM is not
 * commiting any atrocity in terms of precision or any
 * calculation errors. The test simulates a very simple
 * linear movement with constant velocity
 *
 */
TEST(EKF_SLAM, LINEAR_MOVEMENT_INTEGRITY_TEST) {  // This test is not that great, to be improved
  VehicleState *vehicle_state = new VehicleState();
  vehicle_state->last_update = std::chrono::high_resolution_clock::now();
  ImuUpdate *imu_update = new ImuUpdate();
  imu_update->last_update = std::chrono::high_resolution_clock::now();
  Map *track_map = new Map();
  Map initial_map = Map();
  Map *predicted_map = new Map();

  Eigen::Matrix2f Q = Eigen::Matrix2f::Zero();
  Q(0, 0) = 0.1;
  Q(1, 1) = 0.1;
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  R(0, 0) = 0.1;
  R(1, 1) = 0.1;
  R(2, 2) = 0.1;
  MotionModel *motion_model = new ImuVelocityModel(R);
  ObservationModel observation_model = ObservationModel(Q);

  // Initial map
  initial_map.map[Position(0, -2)] = colors::large_orange;
  initial_map.map[Position(0, 2)] = colors::large_orange;
  initial_map.map[Position(2, -2)] = colors::blue;
  initial_map.map[Position(4, -2)] = colors::blue;
  initial_map.map[Position(6, -2)] = colors::blue;
  initial_map.map[Position(7.5, -1.5)] = colors::blue;
  initial_map.map[Position(8.5, -0.5)] = colors::blue;
  initial_map.map[Position(9, 0)] = colors::blue;
  initial_map.map[Position(9, 2)] = colors::blue;
  initial_map.map[Position(9, 4)] = colors::blue;
  initial_map.map[Position(2, 2)] = colors::orange;
  initial_map.map[Position(4, 2)] = colors::orange;
  initial_map.map[Position(6, 2)] = colors::orange;
  initial_map.map[Position(6.5, 3)] = colors::orange;
  initial_map.map[Position(7, 4)] = colors::orange;
  initial_map.map[Position(7, 6)] = colors::orange;
  initial_map.map[Position(7, 6)] = colors::orange;

  ExtendedKalmanFilter *ekf =
      new ExtendedKalmanFilter(vehicle_state, track_map, imu_update, predicted_map, *motion_model,
                               observation_model);  // TODO(marhcouto): put non zero noise matrixes

  for (unsigned int i = 0; i < 10; i++) {
    imu_update->translational_velocity_x = 1;
    imu_update->rotational_velocity = 0;
    imu_update->last_update = std::chrono::high_resolution_clock::now();
    int delta_t = std::chrono::duration_cast<std::chrono::milliseconds>(imu_update->last_update -
                                                                        ekf->get_last_update())
                      .count();
    double delta_x = static_cast<double>(i * delta_t) / 1000;
    ekf->prediction_step();

    predicted_map->map.clear();
    predicted_map->map[Position(0 - delta_x, -2)] = colors::large_orange;
    predicted_map->map[Position(0 - delta_x, 2)] = colors::large_orange;
    predicted_map->map[Position(2 - delta_x, -2)] = colors::blue;
    predicted_map->map[Position(4 - delta_x, -2)] = colors::blue;
    predicted_map->map[Position(6 - delta_x, -2)] = colors::blue;
    predicted_map->map[Position(7.5 - delta_x, -1.5)] = colors::blue;
    predicted_map->map[Position(8.5 - delta_x, -0.5)] = colors::blue;
    predicted_map->map[Position(9 - delta_x, 0)] = colors::blue;
    predicted_map->map[Position(9 - delta_x, 2)] = colors::blue;
    predicted_map->map[Position(9 - delta_x, 4)] = colors::blue;
    predicted_map->map[Position(2 - delta_x, 2)] = colors::orange;
    predicted_map->map[Position(4 - delta_x, 2)] = colors::orange;
    predicted_map->map[Position(6 - delta_x, 2)] = colors::orange;
    predicted_map->map[Position(6.5 - delta_x, 3)] = colors::orange;
    predicted_map->map[Position(7 - delta_x, 4)] = colors::orange;
    predicted_map->map[Position(7 - delta_x, 6)] = colors::orange;
    predicted_map->map[Position(7 - delta_x, 6)] = colors::orange;
    ekf->correction_step();
    ekf->update();

    int orange_count, blue_count, big_orange_count;
    orange_count = 0;
    blue_count = 0;
    big_orange_count = 0;
    for (auto &cone : track_map->map) {
      if (cone.second == colors::orange) {
        orange_count++;
      } else if (cone.second == colors::blue) {
        blue_count++;
      } else if (cone.second == colors::large_orange) {
        big_orange_count++;
      }
      EXPECT_GE(cone.first.x, -1);
      EXPECT_NEAR(cone.first.y, 2, 8);
    }
    // TODO(marhcouto): add more assertions
    // EXPECT_EQ(orange_count, 6);
    // EXPECT_EQ(blue_count, 8);
    // EXPECT_EQ(big_orange_count, 2);
    EXPECT_GE(track_map->map.size(), static_cast<unsigned long int>(16));
    EXPECT_LE(track_map->map.size(), static_cast<unsigned long int>(80));

    EXPECT_GE(vehicle_state->pose.position.x, 0);
    EXPECT_LE(vehicle_state->pose.position.x, 20);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}