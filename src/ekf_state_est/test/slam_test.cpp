// #include <memory>
// #include <thread>

// #include "common_lib/structures/cone.hpp"
// #include "common_lib/structures/position.hpp"
// #include "common_lib/structures/vehicle_state.hpp"
// #include "gtest/gtest.h"
// #include "kalman_filter/ekf.hpp"

// /**
//  * @brief Test the EKF SLAM algorithm in a linear movement
//  *
//  * @details This test aims to check if the EKF SLAM is not
//  * commiting any atrocity in terms of precision or any
//  * calculation errors. The test simulates a very simple
//  * linear movement with constant velocity
//  *
//  */
// TEST(EKF_SLAM, LINEAR_MOVEMENT_INTEGRITY_TEST) {  // This test is not that
//                                                   // great, to be improved
//   std::shared_ptr<common_lib::structures::VehicleState> vehicle_state =
//       std::make_shared<common_lib::structures::VehicleState>();
//   std::shared_ptr<MotionUpdate> motion_update = std::make_shared<MotionUpdate>();
//   std::shared_ptr<std::vector<common_lib::structures::Cone>> track_map =
//       std::make_shared<std::vector<common_lib::structures::Cone>>();
//   std::shared_ptr<std::vector<common_lib::structures::Cone>> initial_map =
//       std::make_shared<std::vector<common_lib::structures::Cone>>();
//   std::shared_ptr<std::vector<common_lib::structures::Cone>> predicted_map =
//       std::make_shared<std::vector<common_lib::structures::Cone>>();
//   motion_update->last_update = rclcpp::Clock().now();

//   Eigen::Matrix2f q_matrix = Eigen::Matrix2f::Zero();
//   q_matrix(0, 0) = static_cast<float>(0.1);
//   q_matrix(1, 1) = static_cast<float>(0.1);
//   Eigen::MatrixXf r_matrix = Eigen::MatrixXf::Zero(5, 5);
//   r_matrix(0, 0) = static_cast<float>(0.1);
//   r_matrix(1, 1) = static_cast<float>(0.1);
//   r_matrix(2, 2) = static_cast<float>(0.1);
//   r_matrix(3, 3) = static_cast<float>(0.1);
//   r_matrix(4, 4) = static_cast<float>(0.1);
//   std::shared_ptr<MotionModel> motion_model = std::make_shared<ImuVelocityModel>(r_matrix);
//   std::shared_ptr<ObservationModel> observation_model =
//       std::make_shared<ObservationModel>(q_matrix);
//   std::shared_ptr<DataAssociationModel> data_association_model =
//       std::make_shared<MaxLikelihood>(71.0);

//   // Initial map
//   initial_map->push_back(
//       common_lib::structures::Cone(common_lib::structures::Position(0, -2),
//                                    common_lib::competition_logic::Color::LARGE_ORANGE, 1.0));
//   initial_map->push_back(
//       common_lib::structures::Cone(common_lib::structures::Position(0, 2),
//                                    common_lib::competition_logic::Color::LARGE_ORANGE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(2, -2), common_lib::competition_logic::Color::BLUE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(4, -2), common_lib::competition_logic::Color::BLUE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(6, -2), common_lib::competition_logic::Color::BLUE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(common_lib::structures::Position(7.5,
//   -1.5),
//                                                       common_lib::competition_logic::Color::BLUE,
//                                                       1.0));
//   initial_map->push_back(common_lib::structures::Cone(common_lib::structures::Position(8.5,
//   -0.5),
//                                                       common_lib::competition_logic::Color::BLUE,
//                                                       1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(9, 0), common_lib::competition_logic::Color::BLUE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(9, 2), common_lib::competition_logic::Color::BLUE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(9, 4), common_lib::competition_logic::Color::BLUE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(2, 2),
//       common_lib::competition_logic::Color::ORANGE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(4, 2),
//       common_lib::competition_logic::Color::ORANGE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(6, 2),
//       common_lib::competition_logic::Color::ORANGE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(6.5, 3),
//       common_lib::competition_logic::Color::ORANGE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(7, 4),
//       common_lib::competition_logic::Color::ORANGE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(7, 6),
//       common_lib::competition_logic::Color::ORANGE, 1.0));
//   initial_map->push_back(common_lib::structures::Cone(
//       common_lib::structures::Position(7, 6),
//       common_lib::competition_logic::Color::ORANGE, 1.0));

//   std::shared_ptr<ExtendedKalmanFilter> ekf = std::make_shared<ExtendedKalmanFilter>(
//       motion_model, observation_model, data_association_model);

//   for (int i = -1; i < 10; i++) {
//     // motion_update->translational_velocity_x = 1;
//     motion_update->rotational_velocity = 0;
//     motion_update->last_update = rclcpp::Clock().now();
//     int delta_t = static_cast<int>(
//         (motion_update->last_update.seconds() - ekf->get_last_update().seconds()) * 1000);
//     double delta_x = static_cast<double>(i * delta_t) / 1000;
//     ekf->prediction_step(*motion_update, "imu");

//     if (i == -1) continue;  // First iteration for setting last update correctly

//     predicted_map->clear();
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(0 - delta_x, -2),
//                                      common_lib::competition_logic::Color::LARGE_ORANGE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(0 - delta_x, 2),
//                                      common_lib::competition_logic::Color::LARGE_ORANGE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(2 - delta_x, -2),
//                                      common_lib::competition_logic::Color::BLUE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(4 - delta_x, -2),
//                                      common_lib::competition_logic::Color::BLUE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(6 - delta_x, -2),
//                                      common_lib::competition_logic::Color::BLUE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(7.5 - delta_x, -1.5),
//                                      common_lib::competition_logic::Color::BLUE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(8.5 - delta_x, -0.5),
//                                      common_lib::competition_logic::Color::BLUE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(9 - delta_x, 0),
//                                      common_lib::competition_logic::Color::BLUE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(9 - delta_x, 2),
//                                      common_lib::competition_logic::Color::BLUE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(9 - delta_x, 4),
//                                      common_lib::competition_logic::Color::BLUE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(2 - delta_x, 2),
//                                      common_lib::competition_logic::Color::ORANGE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(4 - delta_x, 2),
//                                      common_lib::competition_logic::Color::ORANGE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(6 - delta_x, 2),
//                                      common_lib::competition_logic::Color::ORANGE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(6.5 - delta_x, 3),
//                                      common_lib::competition_logic::Color::ORANGE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(7 - delta_x, 4),
//                                      common_lib::competition_logic::Color::ORANGE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(7 - delta_x, 6),
//                                      common_lib::competition_logic::Color::ORANGE, 1.0));
//     predicted_map->push_back(
//         common_lib::structures::Cone(common_lib::structures::Position(7 - delta_x, 6),
//                                      common_lib::competition_logic::Color::ORANGE, 1.0));
//     ekf->correction_step(*predicted_map);
//     ekf->update(vehicle_state, track_map);

//     int orange_count = 0;
//     int blue_count = 0;
//     int big_orange_count = 0;
//     for (const common_lib::structures::Cone &cone : *track_map) {
//       if (cone.color == common_lib::competition_logic::Color::ORANGE) {
//         orange_count++;
//       } else if (cone.color == common_lib::competition_logic::Color::BLUE) {
//         blue_count++;
//       } else if (cone.color == common_lib::competition_logic::Color::LARGE_ORANGE) {
//         big_orange_count++;
//       }
//       EXPECT_GE(cone.position.x, -1);
//       EXPECT_NEAR(cone.position.y, 2, 8);
//     }
//     // TODO(marhcouto): add more assertions
//     // EXPECT_EQ(orange_count, 6);
//     // EXPECT_EQ(blue_count, 8);
//     // EXPECT_EQ(big_orange_count, 2);
//     EXPECT_GE(track_map->size(), static_cast<unsigned long int>(6));
//     EXPECT_LE(track_map->size(), static_cast<unsigned long int>(20));

//     EXPECT_GE(vehicle_state->pose.position.x, -0.5);
//     EXPECT_LE(vehicle_state->pose.position.x, 20);
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   }
//   EXPECT_GE(track_map->size(), static_cast<unsigned long int>(12));
//   EXPECT_LE(track_map->size(), static_cast<unsigned long int>(20));
//   EXPECT_GE(vehicle_state->pose.position.x, -0.5);
//   EXPECT_LE(vehicle_state->pose.position.x, 20);
// }