#include <fstream>  //to write file

#include "gtest/gtest.h"
#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"
// #include <unistd.h> //used to get cwd
// microsegundos - /100
class ExecTimeTest : public ::testing::Test {
 protected:
  VehicleState *vehicle_state_test;
  MotionUpdate *motion_update_test;
  ConeMap *track_map_test;
  ConeMap *perception_map_test;
  Eigen::Matrix2f Q_test;
  Eigen::MatrixXf R_test;
  MotionModel *motion_model_test;
  ObservationModel observation_model_test = ObservationModel(Q_test);
  ExtendedKalmanFilter *ekf_test;
  bool use_odometry_test;
  int helper;
  std::shared_ptr<rclcpp::Node> receiver_publisher_mock;
  std::shared_ptr<rclcpp::Node> lm_node_test;

  std::shared_ptr<custom_interfaces::msg::ConeArray> cone_array_msg;
  std::shared_ptr<rclcpp::Publisher<custom_interfaces::msg::ConeArray>> cones_publisher;

  custom_interfaces::msg::Pose received_pose;            // pose from locmap
  custom_interfaces::msg::ConeArray received_track_map;  // track_map_test from loc map
  std::shared_ptr<rclcpp::Subscription<custom_interfaces::msg::ConeArray>> map_sub;
  std::shared_ptr<rclcpp::Subscription<custom_interfaces::msg::Pose>> localization_sub;

  std::chrono::_V2::system_clock::time_point start_time;
  std::chrono::microseconds duration;
  std::chrono::_V2::system_clock::time_point end_time;
  custom_interfaces::msg::Cone cone_to_send;
  int helper_1;
  bool done;
  void SetUp() override {  // TODO(PedroRomao3) //SetUpTestSuite
    // rclcpp::shutdown();
    start_time = std::chrono::high_resolution_clock::now();
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    helper = 0;
    helper_1 = 0;
    vehicle_state_test = new VehicleState();
    vehicle_state_test->last_update = std::chrono::high_resolution_clock::now();
    motion_update_test = new MotionUpdate();
    motion_update_test->last_update = std::chrono::high_resolution_clock::now();
    track_map_test = new ConeMap();       // Map to publish
    perception_map_test = new ConeMap();  // Map from perception

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
    done = false;
    use_odometry_test = true;

    receiver_publisher_mock = rclcpp::Node::make_shared(
        "cone_array_mock_publisher");  // test node, publishes and receive results from locmap too

    cone_array_msg = std::make_shared<custom_interfaces::msg::ConeArray>();

    cones_publisher = receiver_publisher_mock->create_publisher<custom_interfaces::msg::ConeArray>(
        "perception/cone_coordinates", 10);

    map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
        "track_map", 10, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
          received_track_map = *msg;
          done = true;
          // std::this_thread::sleep_for(std::chrono::seconds(1));
          // rclcpp::shutdown();
        });  // subscribe to track_map topic, get the the map from locmap

    localization_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::Pose>(
        "vehicle_localization", 10, [this](const custom_interfaces::msg::Pose::SharedPtr msg) {
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n LOCALIZATION RECEIVED \n\n");
          received_pose = *msg;
        });

    lm_node_test = std::make_shared<LMNode>(ekf_test, perception_map_test, motion_update_test,
                                            track_map_test, vehicle_state_test, use_odometry_test);
  }
  void TearDown() override {
    receiver_publisher_mock.reset();
    cones_publisher.reset();
    map_sub.reset();
    localization_sub.reset();
    lm_node_test.reset();  // TearDownTestSuite
    delete vehicle_state_test;
    delete motion_update_test;
    delete track_map_test;
    delete perception_map_test;
    // delete motion_model_test;
    delete ekf_test;

    // executor.cancel();
  }
};

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_1CONE) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  cone_to_send.position.x = 1;
  cone_to_send.position.y = 2;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// AFTER ADDING FORTH: %ld ///////\n",
  // cone_array_msg->cone_array.size());
  // complete loc map node
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  cones_publisher->publish(*cone_array_msg);  // send the cones
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();

  while (!done) {
    executor.spin_some();
  }
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// HERE I AM THIS IS ME ///////\n");
  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);  // append
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\nOPEN?: %d\n\n", file.is_open());
  file << "LOC_MAP, all, 1 cones, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)1);
  // executor.cancel();
  // rclcpp::shutdown();
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_3CONES) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  cone_to_send.position.x = 1;
  cone_to_send.position.y = 2;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone
  cone_to_send.position.x = 2;
  cone_to_send.position.y = 4;
  cone_to_send.color = "blue_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone
  cone_to_send.position.x = 4;
  cone_to_send.position.y = 6;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone
  // complete loc map node
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  cones_publisher->publish(*cone_array_msg);          // send the cones
  rclcpp::executors::MultiThreadedExecutor executor;  // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();

  while (!done) {
    executor.spin_some();
  }
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);  // append
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\nOPEN?: %d\n\n", file.is_open());
  file << "LOC_MAP, all, 3 cones, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_50CONES) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  for (int i = 0; i < 50; i++) {
    // Generate random x and y coordinates
    int x = rand() % 50;
    int y = rand() % 50;

    // Set the cone position
    cone_to_send.position.x = x;
    cone_to_send.position.y = y;

    // Set the cone color
    if (i % 2 == 0) {
      cone_to_send.color = "yellow_cone";
    } else {
      cone_to_send.color = "blue_cone";
    }

    // Add the cone to the cone array message
    cone_array_msg->cone_array.push_back(cone_to_send);
  }  // filled my array of cones message with the new cone
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// AFTER ADDING FORTH: %ld ///////\n",
  // cone_array_msg->cone_array.size());
  // complete loc map node
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  cones_publisher->publish(*cone_array_msg);          // send the cones
  rclcpp::executors::MultiThreadedExecutor executor;  // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();

  while (!done) {
    executor.spin_some();
  }
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);  // apped
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
  file << "LOC_MAP, all, 50 cones, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)4);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_100CONES) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  for (int i = 0; i < 100; i++) {
    // Generate random x and y coordinates
    int x = rand() % 50;
    int y = rand() % 50;

    // Set the cone position
    cone_to_send.position.x = x;
    cone_to_send.position.y = y;

    // Set the cone color
    if (i % 2 == 0) {
      cone_to_send.color = "yellow_cone";
    } else {
      cone_to_send.color = "blue_cone";
    }

    // Add the cone to the cone array message
    cone_array_msg->cone_array.push_back(cone_to_send);
  }

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// BEFORE ADDING FORTH: %ld ///////\n",
  // cone_array_msg->cone_array.size());
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone
  // complete loc map node
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  cones_publisher->publish(*cone_array_msg);          // send the cones
  rclcpp::executors::MultiThreadedExecutor executor;  // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();

  while (!done) {
    executor.spin_some();
  }
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);  // apped
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
  file << "LOC_MAP, all, 100 cones, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_200CONES) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  for (int i = 0; i < 200; i++) {
    // Generate random x and y coordinates
    int x = rand() % 50;
    int y = rand() % 50;

    // Set the cone position
    cone_to_send.position.x = x;
    cone_to_send.position.y = y;

    // Set the cone color
    if (i % 2 == 0) {
      cone_to_send.color = "yellow_cone";
    } else {
      cone_to_send.color = "blue_cone";
    }

    // Add the cone to the cone array message
    cone_array_msg->cone_array.push_back(cone_to_send);
  }

  // complete loc map node
  cones_publisher->publish(*cone_array_msg);          // send the cones
  rclcpp::executors::MultiThreadedExecutor executor;  // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();

  while (!done) {
    executor.spin_some();
  }
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);  // apped
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
  file << "LOC_MAP, all, 200 cones, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

// PERFORMANCE TESTS

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_1X100CONE) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  cone_to_send.position.x = 1;
  cone_to_send.position.y = 2;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone

  map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 1, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
        received_track_map = *msg;
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        end_time = std::chrono::high_resolution_clock::now();
        duration = (duration +
                    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
                    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time));
        start_time = std::chrono::high_resolution_clock::now();
        cones_publisher->publish(*cone_array_msg);
        helper++;
        if (helper == 100) {
          duration = duration / 100;
          std::ofstream file("../../src/loc_map/test/integration_test.csv",
                             std::ios::app);  // apped
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
          file << "LOC_MAP, all, 1X100 cones, " << duration.count() << "\n";
          file.close();
          done = true;
        }
        // rclcpp::shutdown();
      });

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// BEFORE ADDING FORTH: %ld ///////\n",
  // cone_array_msg->cone_array.size());
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// AFTER ADDING FORTH: %ld ///////\n",
  // cone_array_msg->cone_array.size());
  // complete loc map node
  // std::this_thread::sleep_for(std::chrono::seconds(1));

  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone

  cones_publisher->publish(*cone_array_msg);
  start_time = std::chrono::high_resolution_clock::now();

  // rclcpp::shutdown();
  //  send the cones
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();
  while (!done) {
    executor.spin_some();
  }

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)1);
  // executor.cancel();
  // rclcpp::shutdown();
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_5X100CONE) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  cone_to_send.position.x = 1;  // to 50
  cone_to_send.position.y = 2;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone

  cone_to_send.position.x = 5;
  cone_to_send.position.y = 9;
  cone_to_send.color = "blue_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone

  cone_to_send.position.x = 13;
  cone_to_send.position.y = 16;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone

  cone_to_send.position.x = 16;
  cone_to_send.position.y = 19;
  cone_to_send.color = "blue_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 23;
  cone_to_send.position.y = 26;
  cone_to_send.color = "blue_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send);

  map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 1, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
        received_track_map = *msg;
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        end_time = std::chrono::high_resolution_clock::now();
        duration = (duration +
                    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
                    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time));
        start_time = std::chrono::high_resolution_clock::now();
        cones_publisher->publish(*cone_array_msg);
        helper++;
        if (helper == 100) {
          duration = duration / 100;
          std::ofstream file("../../src/loc_map/test/integration_test.csv",
                             std::ios::app);  // apped
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
          file << "LOC_MAP, all, 5X100 cones, " << duration.count() << "\n";
          file.close();
          done = true;
        }
        // rclcpp::shutdown();
      });

  // complete loc map node
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  cones_publisher->publish(*cone_array_msg);          // send the cones
  rclcpp::executors::MultiThreadedExecutor executor;  // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();
  while (!done) {
    executor.spin_some();
  }
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
  // executor.cancel();
  // rclcpp::shutdown();
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_50X100CONE) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 1, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
        received_track_map = *msg;
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        end_time = std::chrono::high_resolution_clock::now();
        duration = (duration +
                    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
                    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time));
        start_time = std::chrono::high_resolution_clock::now();
        cones_publisher->publish(*cone_array_msg);
        helper++;
        if (helper == 100) {
          duration = duration / 100;
          std::ofstream file("../../src/loc_map/test/integration_test.csv",
                             std::ios::app);  // apped
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
          file << "LOC_MAP, all, 50X100 cones, " << duration.count() << "\n";
          file.close();
          done = true;
        }
        // rclcpp::shutdown();
      });

  cone_to_send.position.x = 1;
  cone_to_send.position.y = 2;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send);
  for (int i = 0; i < 50; i++) {
    // Generate random x and y coordinates
    int x = rand() % 50;
    int y = rand() % 50;

    // Set the cone position
    cone_to_send.position.x = x;
    cone_to_send.position.y = y;

    // Set the cone color
    if (i % 2 == 0) {
      cone_to_send.color = "yellow_cone";
    } else {
      cone_to_send.color = "blue_cone";
    }

    // Add the cone to the cone array message
    cone_array_msg->cone_array.push_back(cone_to_send);
  }

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// BEFORE ADDING FORTH: %ld ///////\n",
  // cone_array_msg->cone_array.size());  // filled my array of cones message with the new cone
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// AFTER ADDING FORTH: %ld ///////\n",
  // cone_array_msg->cone_array.size());
  // complete loc map node
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  cones_publisher->publish(*cone_array_msg);          // send the cones
  rclcpp::executors::MultiThreadedExecutor executor;  // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();
  while (!done) {
    executor.spin_some();
  }
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }
  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)4);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_100X100CONE) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 1, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
        received_track_map = *msg;
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        end_time = std::chrono::high_resolution_clock::now();
        duration = (duration +
                    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
                    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time));
        start_time = std::chrono::high_resolution_clock::now();
        cones_publisher->publish(*cone_array_msg);
        helper++;
        if (helper == 100) {
          duration = duration / 100;
          std::ofstream file("../../src/loc_map/test/integration_test.csv",
                             std::ios::app);  // apped
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
          file << "LOC_MAP, all, 100X100 cones, " << duration.count() << "\n";
          file.close();
          done = true;
        }
        // rclcpp::shutdown();
      });

  cone_to_send.position.x = 1;
  cone_to_send.position.y = 2;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send);
  for (int i = 0; i < 100; i++) {
    // Generate random x and y coordinates
    int x = rand() % 50;
    int y = rand() % 50;

    // Set the cone position
    cone_to_send.position.x = x;
    cone_to_send.position.y = y;

    // Set the cone color
    if (i % 2 == 0) {
      cone_to_send.color = "yellow_cone";
    } else {
      cone_to_send.color = "blue_cone";
    }

    // Add the cone to the cone array message
    cone_array_msg->cone_array.push_back(cone_to_send);
  }

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// BEFORE ADDING FORTH: %ld ///////\n",
  // cone_array_msg->cone_array.size());  // filled my array of cones message with the new cone
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// AFTER ADDING FORTH: %ld ///////\n",
  // cone_array_msg->cone_array.size());
  // complete loc map node
  // std::this_thread::sleep_for(std::chrono::seconds(1));

  cones_publisher->publish(*cone_array_msg);          // send the cones
  rclcpp::executors::MultiThreadedExecutor executor;  // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();
  while (!done) {
    executor.spin_some();
  }
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)4);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_200X100CONE) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 1, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
        received_track_map = *msg;
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        end_time = std::chrono::high_resolution_clock::now();
        duration = (duration +
                    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n DURATION STEP: %ld  \n",
                    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time));
        start_time = std::chrono::high_resolution_clock::now();
        cones_publisher->publish(*cone_array_msg);
        helper++;
        if (helper == 100) {
          duration = duration / 100;
          std::ofstream file("../../src/loc_map/test/integration_test.csv",
                             std::ios::app);  // apped
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
          file << "LOC_MAP, all, 200X100 cones, " << duration.count() << "\n";
          file.close();
          done = true;
        }
        // rclcpp::shutdown();
      });

  cone_to_send.position.x = 1;
  cone_to_send.position.y = 2;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send);
  for (int i = 0; i < 200; i++) {
    // Generate random x and y coordinates
    int x = rand() % 50;
    int y = rand() % 50;

    // Set the cone position
    cone_to_send.position.x = x;
    cone_to_send.position.y = y;

    // Set the cone color
    if (i % 2 == 0) {
      cone_to_send.color = "yellow_cone";
    } else {
      cone_to_send.color = "blue_cone";
    }

    // Add the cone to the cone array message
    cone_array_msg->cone_array.push_back(cone_to_send);
  }

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// BEFORE ADDING FORTH: %ld ///////\n",
  // cone_array_msg->cone_array.size());  // filled my array of cones message with the new cone
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// AFTER ADDING FORTH: %ld ///////\n",
  // cone_array_msg->cone_array.size());
  // complete loc map node
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  cones_publisher->publish(*cone_array_msg);          // send the cones
  rclcpp::executors::MultiThreadedExecutor executor;  // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();
  while (!done) {
    executor.spin_some();
  }
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)4);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_5CONES_AFTER_5) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info
  map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 10, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
        received_track_map = *msg;
        if (helper > 0)
          done = true;
        else {
          helper++;
          cone_array_msg->cone_array.clear();
          cone_to_send.position.x = 4;
          cone_to_send.position.y = 6;
          cone_to_send.color = "blue_cone";  // colors::yellow;//created a cone
          cone_array_msg->cone_array.push_back(
              cone_to_send);  // filled my array of cones message with the new cone
          cone_to_send.position.x = 8;
          cone_to_send.position.y = 6;
          cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
          cone_array_msg->cone_array.push_back(
              cone_to_send);  // filled my array of cones message with the new cone
          cone_to_send.position.x = 6;
          cone_to_send.position.y = 8;
          cone_to_send.color = "blue_cone";  // colors::yellow;//created a cone
          cone_array_msg->cone_array.push_back(
              cone_to_send);  // filled my array of cones message with the new cone
          cone_to_send.position.x = 10;
          cone_to_send.position.y = 8;
          cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
          cone_array_msg->cone_array.push_back(cone_to_send);
          cone_to_send.position.x = 8;
          cone_to_send.position.y = 10;
          cone_to_send.color = "blue_cone";  // colors::yellow;//created a cone
          cone_array_msg->cone_array.push_back(
              cone_to_send);  // filled my array of cones message with the new cone
          // complete loc map node
          // std::this_thread::sleep_for(std::chrono::seconds(1));
          cones_publisher->publish(*cone_array_msg);
        }

        // std::this_thread::sleep_for(std::chrono::seconds(1));
        // rclcpp::shutdown();
      });
  cone_to_send.position.x = 2;
  cone_to_send.position.y = 0;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone
  cone_to_send.position.x = 0;
  cone_to_send.position.y = 2;
  cone_to_send.color = "blue_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone
  cone_to_send.position.x = 4;
  cone_to_send.position.y = 2;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone
  cone_to_send.position.x = 2;
  cone_to_send.position.y = 4;
  cone_to_send.color = "blue_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone
  cone_to_send.position.x = 6;
  cone_to_send.position.y = 4;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send);
  // complete loc map node
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  cones_publisher->publish(*cone_array_msg);          // send the cones
  rclcpp::executors::MultiThreadedExecutor executor;  // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();

  while (!done) {
    executor.spin_some();
  }
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);  // append
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\nOPEN?: %d\n\n", file.is_open());
  file << "LOC_MAP, all,5 after 5, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_50CONES_AFTER_50) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info
  map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 10, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
        received_track_map = *msg;
        if (helper > 0)
          done = true;
        else {
          helper++;
          cone_array_msg->cone_array.clear();
          for (int i = 0; i < 50; i++) {
            // Generate random x and y coordinates
            int x = rand() % 50;
            int y = rand() % 50;  // TODO(PedroRomao3) Improve repeated code

            // Set the cone position
            cone_to_send.position.x = x;
            cone_to_send.position.y = y;

            // Set the cone color
            if (i % 2 == 0) {
              cone_to_send.color = "yellow_cone";
            } else {
              cone_to_send.color = "blue_cone";
            }

            // Add the cone to the cone array message
            cone_array_msg->cone_array.push_back(cone_to_send);
          }
          // complete loc map node
          // std::this_thread::sleep_for(std::chrono::seconds(1));
          cones_publisher->publish(*cone_array_msg);
        }

        // std::this_thread::sleep_for(std::chrono::seconds(1));
        // rclcpp::shutdown();
      });
  for (int i = 0; i < 50; i++) {
    // Generate random x and y coordinates
    int x = rand() % 50;
    int y = rand() % 50;

    // Set the cone position
    cone_to_send.position.x = x;
    cone_to_send.position.y = y;

    // Set the cone color
    if (i % 2 == 0) {
      cone_to_send.color = "yellow_cone";
    } else {
      cone_to_send.color = "blue_cone";
    }

    // Add the cone to the cone array message
    cone_array_msg->cone_array.push_back(cone_to_send);
  }
  // complete loc map node
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  cones_publisher->publish(*cone_array_msg);          // send the cones
  rclcpp::executors::MultiThreadedExecutor executor;  // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();

  while (!done) {
    executor.spin_some();
  }
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);  // append
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\nOPEN?: %d\n\n", file.is_open());
  file << "LOC_MAP, all,50 after 50, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_100CONES_AFTER_100) {
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) !=
  // RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info
  map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 10, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
        received_track_map = *msg;
        if (helper > 0)
          done = true;
        else {
          helper++;
          cone_array_msg->cone_array.clear();
          for (int i = 0; i < 100; i++) {
            // Generate random x and y coordinates
            int x = rand() % 50;
            int y = rand() % 50;

            // Set the cone position
            cone_to_send.position.x = x;
            cone_to_send.position.y = y;

            // Set the cone color
            if (i % 2 == 0) {
              cone_to_send.color = "yellow_cone";
            } else {
              cone_to_send.color = "blue_cone";
            }

            // Add the cone to the cone array message
            cone_array_msg->cone_array.push_back(cone_to_send);
          }
          // complete loc map node
          // std::this_thread::sleep_for(std::chrono::seconds(1));
          cones_publisher->publish(*cone_array_msg);
        }

        // std::this_thread::sleep_for(std::chrono::seconds(1));
        // rclcpp::shutdown();
      });
  for (int i = 0; i < 100; i++) {
    // Generate random x and y coordinates
    int x = rand() % 50;
    int y = rand() % 50;

    // Set the cone position
    cone_to_send.position.x = x;
    cone_to_send.position.y = y;

    // Set the cone color
    if (i % 2 == 0) {
      cone_to_send.color = "yellow_cone";
    } else {
      cone_to_send.color = "blue_cone";
    }

    // Add the cone to the cone array message
    cone_array_msg->cone_array.push_back(cone_to_send);
  }
  // complete loc map node
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  cones_publisher->publish(*cone_array_msg);          // send the cones
  rclcpp::executors::MultiThreadedExecutor executor;  // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node_test);
  start_time = std::chrono::high_resolution_clock::now();

  while (!done) {
    executor.spin_some();
  }
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());

  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);
  file << "LOC_MAP, all,100 after 100, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_CORR_10) {
  // create cone map , maybe state

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->correction_step();

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);
  file << "LOC_MAP, EKF Correction Step, 10, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_CORR_50) {
  // create cone map , maybe state

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->correction_step();

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);
  file << "LOC_MAP, EKF Correction Step, 50, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_CORR_100) {
  // create cone map , maybe state

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->correction_step();

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);
  file << "LOC_MAP, EKF Correction Step, 100, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}
TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_PRED_10) {
  // create motion update and state with adequate workload

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->prediction_step();

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);
  file << "LOC_MAP, EKF Prediction Step, 10, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}
TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_PRED_50) {
  // create motion update and state with adequate workload

  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->prediction_step();

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);
  file << "LOC_MAP, EKF Prediction Step, 50, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}
TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_PRED_100) {
  // create motion update and state with adequate workload
  start_time = std::chrono::high_resolution_clock::now();
  ekf_test->prediction_step();

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);
  file << "LOC_MAP, EKF Prediction Step, 100, " << duration.count() << "\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}
// TODO(PedroRomao3)
// EKF steps
