#include <unistd.h>

#include <fstream>  //to write file

#include "gtest/gtest.h"
#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"
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
  std::string workload;
  void print_to_file() {
    std::ofstream file(
        "../../src/performance/exec_time/loc_map.csv", std::ios::app);  // append
    file << "LOC_MAP, " << workload << ", " << duration.count() << "\n";
    file.close();
  }

  void fill_X(int size) {
    for (int i = 3; i <= size; i++) {
      double randomX = ((double)rand() / RAND_MAX) * 50.0;
      double randomY = ((double)rand() / RAND_MAX) * 50.0;

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

  void fill_cone_array(int size) {
    for (int i = 0; i < size; i++) {
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
  }

  void SetUp() override {  // TODO(PedroRomao3) //SetUpTestSuite
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
  char cwd[PATH_MAX];
  if (getcwd(cwd, sizeof(cwd)) != NULL) {
    printf("\n CWD:%s \n", cwd);
  }
  cone_to_send.position.x = 1;
  cone_to_send.position.y = 2;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(
      cone_to_send);  // filled my array of cones message with the new cone

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

  workload = "all,1 CONE";
  print_to_file();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)1);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_3CONES) {
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

  workload = "all,3 CONES";
  print_to_file();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_50CONES) {
  fill_cone_array(50);

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

  workload = "all,50 CONES";
  print_to_file();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)4);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_100CONES) {
  fill_cone_array(100);

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

  workload = "all,100 CONES";
  print_to_file();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_200CONES) {
  fill_cone_array(200);

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

  workload = "all,200 CONES";
  print_to_file();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

// PERFORMANCE TESTS

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_1X100CONE) {
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
          workload = "all,1 X 100 CONES";
          print_to_file();
          done = true;
        }
      });

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

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)1);
  // executor.cancel();
  // rclcpp::shutdown();
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_5X100CONE) {
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
          workload = "all,5 X 100 CONES";
          print_to_file();
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

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
  // executor.cancel();
  // rclcpp::shutdown();
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_50X100CONE) {
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
          workload = "all,50 X 100 CONES";
          print_to_file();
          done = true;
        }
        // rclcpp::shutdown();
      });

  cone_to_send.position.x = 1;
  cone_to_send.position.y = 2;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send);
  fill_cone_array(50);

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

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)4);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_100X100CONE) {
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
          workload = "all,100 X 100 CONES";
          print_to_file();
          done = true;
        }
        // rclcpp::shutdown();
      });

  cone_to_send.position.x = 1;
  cone_to_send.position.y = 2;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send);
  fill_cone_array(100);

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

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)4);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_200X100CONE) {
  map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 1, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
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
          workload = "all,200 X 100 CONES";
          print_to_file();
          done = true;
        }
        // rclcpp::shutdown();
      });

  cone_to_send.position.x = 1;
  cone_to_send.position.y = 2;
  cone_to_send.color = "yellow_cone";  // colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send);
  fill_cone_array(200);
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

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)4);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_5CONES_AFTER_5) {
  map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 10, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
        received_track_map = *msg;
        if (helper > 0) {
          done = true;
        } else {
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

  workload = "all,5 AFTER 5 CONES";
  print_to_file();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_50CONES_AFTER_50) {
  map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 10, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
        received_track_map = *msg;
        if (helper > 0) {
          done = true;
        } else {
          helper++;
          cone_array_msg->cone_array.clear();
          fill_cone_array(50);
          // complete loc map node
          // std::this_thread::sleep_for(std::chrono::seconds(1));
          cones_publisher->publish(*cone_array_msg);
        }

        // std::this_thread::sleep_for(std::chrono::seconds(1));
        // rclcpp::shutdown();
      });
  fill_cone_array(50);
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

  workload = "all,50 AFTER 50";
  print_to_file();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_100CONES_AFTER_100) {
  map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
      "track_map", 10, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
        received_track_map = *msg;
        if (helper > 0) {
          done = true;
        } else {
          helper++;
          cone_array_msg->cone_array.clear();
          fill_cone_array(100);
          // complete loc map node
          // std::this_thread::sleep_for(std::chrono::seconds(1));
          cones_publisher->publish(*cone_array_msg);
        }

        // std::this_thread::sleep_for(std::chrono::seconds(1));
        // rclcpp::shutdown();
      });
  fill_cone_array(100);
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

  workload = "all,100 AFTER 100 CONES";
  print_to_file();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_PRED_10) {
  // create motion update and state with adequate workload

  motion_update_test->translational_velocity = 1.58113883;
  motion_update_test->translational_velocity_x = 1.5;
  motion_update_test->translational_velocity_y = 0.5;
  motion_update_test->rotational_velocity = 6.0;
  motion_update_test->steering_angle = 2.0;
  motion_update_test->last_update = std::chrono::high_resolution_clock::now();
  ekf_test->set_X(23);
  ekf_test->set_P(23);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n HERE I AM THIS IS ME#1 \n\n");
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n HERE I AM THIS IS ME#2 \n\n");
  // necessary?
  ekf_test->set_X_y(3, -1.637208342552185);
  ekf_test->set_X_y(4, 14.400202751159668);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n HERE I AM THIS IS ME#3 \n\n");
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(5, -2.216218948364258);
  ekf_test->set_X_y(6, 11.487205505371094);
  ekf_test->push_to_colors(colors::Color::blue);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n HERE I AM THIS IS ME#4 \n\n");
  ekf_test->set_X_y(7, -3.867227792739868);
  ekf_test->set_X_y(8, 9.018211364746094);
  ekf_test->push_to_colors(colors::Color::blue);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n HERE I AM THIS IS ME#5 \n\n");

  ekf_test->set_X_y(9, -6.336233615875244);
  ekf_test->set_X_y(10, 7.367220401763916);
  ekf_test->push_to_colors(colors::Color::blue);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n HERE I AM THIS IS ME#6 \n\n");

  ekf_test->set_X_y(11, -9.250235557556152);
  ekf_test->set_X_y(12, 6.788230895996094);
  ekf_test->push_to_colors(colors::Color::blue);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n HERE I AM THIS IS ME#7 \n\n");

  ekf_test->set_X_y(13, 16.861791610717773);
  ekf_test->set_X_y(14, 14.40013599395752);
  ekf_test->push_to_colors(colors::Color::yellow);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n HERE I AM THIS IS ME#8 \n\n");

  ekf_test->set_X_y(15, 16.28278160095215);
  ekf_test->set_X_y(16, 11.487138748168945);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(17, 14.6317720413208);
  ekf_test->set_X_y(18, 9.018143653869629);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(19, 12.162766456604004);
  ekf_test->set_X_y(20, 7.367153644561768);
  ekf_test->push_to_colors(colors::Color::yellow);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n HERE I AM THIS IS ME#9 \n\n");
  ekf_test->set_X_y(21, 9.249764442443848);
  ekf_test->set_X_y(22, 6.788164138793945);
  ekf_test->push_to_colors(colors::Color::yellow);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n HERE I AM THIS IS ME#10 \n\n");
  //\necessary?

  start_time = std::chrono::high_resolution_clock::now();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n HERE I AM THIS IS ME#10 \n\n");
  ekf_test->prediction_step(*motion_update_test);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n HERE I AM THIS IS ME#11 \n\n");
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF pred step, 10";
  print_to_file();

  // EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}
TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_PRED_50) {
  // create motion update and state with adequate workload

  motion_update_test->translational_velocity = 1.58113883;
  motion_update_test->translational_velocity_x = 1.5;
  motion_update_test->translational_velocity_y = 0.5;
  motion_update_test->rotational_velocity = 6.0;
  motion_update_test->steering_angle = 2.0;
  motion_update_test->last_update = std::chrono::high_resolution_clock::now();
  ekf_test->set_X(103);
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

  // EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}
TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_PRED_100) {
  // create motion update and state with adequate workload(size)

  ekf_test->set_X(203);
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

  // EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_CORR_10) {
  ConeMap coneMap;
  for (int i = 0; i < 10; i++) {
    Position conePosition(i * 2.0, i * 2.0);

    // Add the cone to the map
    coneMap.map[conePosition] = colors::blue;
  }
  coneMap.last_update = std::chrono::high_resolution_clock::now();

  ekf_test->set_X(23);
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
  ekf_test->correction_step(coneMap);

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF Correction Step, 10 and 10 From \"perception\",";
  print_to_file();

  // EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_CORR_50) {
  // create cone map , maybe state
  ConeMap coneMap;
  for (int i = 0; i < 10; i++) {
    Position conePosition(i * 2.0, i * 2.0);

    // Add the cone to the map
    coneMap.map[conePosition] = colors::blue;
  }
  coneMap.last_update = std::chrono::high_resolution_clock::now();

  ekf_test->set_X(103);
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

  workload = "EKF Correction Step, 50 and 10 From \"perception\",";
  print_to_file();

  // EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_EKF_CORR_100) {
  // create cone map , maybe state
  ConeMap coneMap;
  for (int i = 0; i < 10; i++) {
    Position conePosition(i * 2.0, i * 2.0);

    // Add the cone to the map
    coneMap.map[conePosition] = colors::blue;
  }
  coneMap.last_update = std::chrono::high_resolution_clock::now();
  ekf_test->set_X(203);
  ekf_test->set_P(203);
  ekf_test->set_X_y(0, -15.0);
  ekf_test->set_X_y(1, 0.0);
  ekf_test->set_X_y(2, 0.0);
  fill_X(202);

  start_time = std::chrono::high_resolution_clock::now();

  ekf_test->correction_step(coneMap);

  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  workload = "EKF Correction Step, 100 and 10 From \"perception\",";
  print_to_file();

  // EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int)3);
}

// TODO(PedroRomao3)
// imrpove EKF step tests
// clean up code and remove duplicate code

// // necessary?
// ekf_test->set_X_y(3, -1.637208342552185);
// ekf_test->set_X_y(4, 14.400202751159668);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(5, -2.216218948364258);
// ekf_test->set_X_y(6, 11.487205505371094);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(7, -3.867227792739868);
// ekf_test->set_X_y(8, 9.018211364746094);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(9, -6.336233615875244);
// ekf_test->set_X_y(10, 7.367220401763916);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(11, -9.250235557556152);
// ekf_test->set_X_y(12, 6.788230895996094);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(13, -12.160233497619629);
// ekf_test->set_X_y(14, 7.367241382598877);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(15, -14.630228042602539);
// ekf_test->set_X_y(16, 9.018250465393066);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(17, -16.28021812438965);
// ekf_test->set_X_y(18, 11.48725700378418);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(19, -16.86020851135254);
// ekf_test->set_X_y(20, 14.40025806427002);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(21, -16.280197143554688);
// ekf_test->set_X_y(22, 17.313356399536133);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(23, -14.630188941955566);
// ekf_test->set_X_y(24, 19.783050537109375);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(25, -12.16018295288086);
// ekf_test->set_X_y(26, 21.43324089050293);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(27, -9.2501802444458);
// ekf_test->set_X_y(28, 22.012731552124023);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(29, -6.336182594299316);
// ekf_test->set_X_y(30, 21.43321990966797);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(31, -3.8671889305114746);
// ekf_test->set_X_y(32, 19.783010482788086);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(33, -2.2161977291107178);
// ekf_test->set_X_y(34, 17.313304901123047);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(35, 1.7456188201904297);
// ekf_test->set_X_y(36, 21.9042911529541);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(37, 5.1885271072387695);
// ekf_test->set_X_y(38, 24.20477867126465);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(39, 9.24983024597168);
// ekf_test->set_X_y(40, 25.012163162231445);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(41, 13.310827255249023);
// ekf_test->set_X_y(42, 24.204750061035156);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(43, 16.753820419311523);
// ekf_test->set_X_y(44, 21.90423583984375);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(45, 19.05380630493164);
// ekf_test->set_X_y(46, 18.461328506469727);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(47, 19.861791610717773);
// ekf_test->set_X_y(48, 14.400125503540039);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(49, 19.05377769470215);
// ekf_test->set_X_y(50, 10.339128494262695);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(51, 16.753765106201172);
// ekf_test->set_X_y(52, 6.89613676071167);
// ekf_test->push_to_colors(colors::Color::blue);
// ekf_test->set_X_y(53, 16.861791610717773);
// ekf_test->set_X_y(54, 14.40013599395752);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(55, 16.28278160095215);
// ekf_test->set_X_y(56, 11.487138748168945);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(57, 14.6317720413208);
// ekf_test->set_X_y(58, 9.018143653869629);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(59, 12.162766456604004);
// ekf_test->set_X_y(60, 7.367153644561768);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(61, 9.249764442443848);
// ekf_test->set_X_y(62, 6.788164138793945);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(63, 6.33656644821167);
// ekf_test->set_X_y(64, 7.3671746253967285);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(65, 3.8668723106384277);
// ekf_test->set_X_y(66, 9.018182754516602);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(67, 2.2166810035705566);
// ekf_test->set_X_y(68, 11.487190246582031);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(69, 1.637291669845581);
// ekf_test->set_X_y(70, 14.400191307067871);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(71, 2.2167022228240967);
// ekf_test->set_X_y(72, 17.313289642333984);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(73, 3.8669111728668213);
// ekf_test->set_X_y(74, 19.782983779907227);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(75, 6.336617469787598);
// ekf_test->set_X_y(76, 21.43317413330078);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(77, 9.2498197555542);
// ekf_test->set_X_y(78, 22.012662887573242);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(79, 12.162817001342773);
// ekf_test->set_X_y(80, 21.43315315246582);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(81, 14.631811141967773);
// ekf_test->set_X_y(82, 19.782943725585938);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(83, 16.28280258178711);
// ekf_test->set_X_y(84, 17.3132381439209);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(85, -1.7451812028884888);
// ekf_test->set_X_y(86, 21.9043025970459);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(87, -5.188172817230225);
// ekf_test->set_X_y(88, 24.204816818237305);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(89, -9.25016975402832);
// ekf_test->set_X_y(90, 25.012231826782227);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(91, -13.310173034667969);
// ekf_test->set_X_y(92, 24.204845428466797);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(93, -16.750181198120117);
// ekf_test->set_X_y(94, 21.90435791015625);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(95, -19.050193786621094);
// ekf_test->set_X_y(96, 18.46146583557129);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(97, -19.86020851135254);
// ekf_test->set_X_y(98, 14.4002685546875);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(99, -19.050222396850586);
// ekf_test->set_X_y(100, 10.339265823364258);
// ekf_test->push_to_colors(colors::Color::yellow);
// ekf_test->set_X_y(101, -16.75023651123047);
// ekf_test->set_X_y(102, 6.8962578773498535);
// ekf_test->push_to_colors(colors::Color::yellow);

//\necessary?
/*

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
  ekf_test->set_X_y(13, -12.160233497619629);
  ekf_test->set_X_y(14, 7.367241382598877);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(15, -14.630228042602539);
  ekf_test->set_X_y(16, 9.018250465393066);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(17, -16.28021812438965);
  ekf_test->set_X_y(18, 11.48725700378418);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(19, -16.86020851135254);
  ekf_test->set_X_y(20, 14.40025806427002);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(21, -16.280197143554688);
  ekf_test->set_X_y(22, 17.313356399536133);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(23, -14.630188941955566);
  ekf_test->set_X_y(24, 19.783050537109375);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(25, -12.16018295288086);
  ekf_test->set_X_y(26, 21.43324089050293);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(27, -9.2501802444458);
  ekf_test->set_X_y(28, 22.012731552124023);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(29, -6.336182594299316);
  ekf_test->set_X_y(30, 21.43321990966797);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(31, -3.8671889305114746);
  ekf_test->set_X_y(32, 19.783010482788086);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(33, -2.2161977291107178);
  ekf_test->set_X_y(34, 17.313304901123047);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(35, 1.7456188201904297);
  ekf_test->set_X_y(36, 21.9042911529541);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(37, 5.1885271072387695);
  ekf_test->set_X_y(38, 24.20477867126465);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(39, 9.24983024597168);
  ekf_test->set_X_y(40, 25.012163162231445);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(41, 13.310827255249023);
  ekf_test->set_X_y(42, 24.204750061035156);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(43, 16.753820419311523);
  ekf_test->set_X_y(44, 21.90423583984375);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(45, 19.05380630493164);
  ekf_test->set_X_y(46, 18.461328506469727);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(47, 19.861791610717773);
  ekf_test->set_X_y(48, 14.400125503540039);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(49, 19.05377769470215);
  ekf_test->set_X_y(50, 10.339128494262695);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(51, 16.753765106201172);
  ekf_test->set_X_y(52, 6.89613676071167);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(53, 13.310755729675293);
  ekf_test->set_X_y(54, 4.596149444580078);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(55, 9.24975299835205);
  ekf_test->set_X_y(56, 3.790163993835449);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(57, 5.188456058502197);
  ekf_test->set_X_y(58, 4.596179008483887);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(59, 1.7455644607543945);
  ekf_test->set_X_y(60, 6.896191120147705);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(61, -1.637208342552185);
  ekf_test->set_X_y(62, 14.400202751159668);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(63, 16.861791610717773);
  ekf_test->set_X_y(64, 14.40013599395752);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(65, 16.28278160095215);
  ekf_test->set_X_y(66, 11.487138748168945);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(67, 14.6317720413208);
  ekf_test->set_X_y(68, 9.018143653869629);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(69, 12.162766456604004);
  ekf_test->set_X_y(70, 7.367153644561768);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(71, 9.249764442443848);
  ekf_test->set_X_y(72, 6.788164138793945);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(73, 6.33656644821167);
  ekf_test->set_X_y(74, 7.3671746253967285);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(75, 3.8668723106384277);
  ekf_test->set_X_y(76, 9.018182754516602);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(77, 2.2166810035705566);
  ekf_test->set_X_y(78, 11.487190246582031);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(79, 1.637291669845581);
  ekf_test->set_X_y(80, 14.400191307067871);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(81, 2.2167022228240967);
  ekf_test->set_X_y(82, 17.313289642333984);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(83, 3.8669111728668213);
  ekf_test->set_X_y(84, 19.782983779907227);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(85, 6.336617469787598);
  ekf_test->set_X_y(86, 21.43317413330078);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(87, 9.2498197555542);
  ekf_test->set_X_y(88, 22.012662887573242);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(89, 12.162817001342773);
  ekf_test->set_X_y(90, 21.43315315246582);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(91, 14.631811141967773);
  ekf_test->set_X_y(92, 19.782943725585938);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(93, 16.28280258178711);
  ekf_test->set_X_y(94, 17.3132381439209);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(95, -1.7451812028884888);
  ekf_test->set_X_y(96, 21.9043025970459);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(97, -5.188172817230225);
  ekf_test->set_X_y(98, 24.204816818237305);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(99, -9.25016975402832);
  ekf_test->set_X_y(100, 25.012231826782227);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(101, -13.310173034667969);
  ekf_test->set_X_y(102, 24.204845428466797);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(103, -16.750181198120117);
  ekf_test->set_X_y(104, 21.90435791015625);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(105, -19.050193786621094);
  ekf_test->set_X_y(106, 18.46146583557129);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(107, -19.86020851135254);
  ekf_test->set_X_y(108, 14.4002685546875);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(109, -19.050222396850586);
  ekf_test->set_X_y(110, 10.339265823364258);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(111, -16.75023651123047);
  ekf_test->set_X_y(112, 6.8962578773498535);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(113, -13.3102445602417);
  ekf_test->set_X_y(114, 4.600245475769043);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(115, -9.25024700164795);
  ekf_test->set_X_y(116, 3.7902307510375977);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(117, -5.188243865966797);
  ekf_test->set_X_y(118, 4.596216201782227);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(119, -1.745235562324524);
  ekf_test->set_X_y(120, 6.896203517913818);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(121, 16.861791610717773);
  ekf_test->set_X_y(122, 14.40013599395752);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(123, -1.6372408866882324);
  ekf_test->set_X_y(124, 5.400203227996826);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(125, -1.637248158454895);
  ekf_test->set_X_y(126, 3.4002034664154053);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(127, 1.6372591257095337);
  ekf_test->set_X_y(128, 5.400191783905029);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(129, 1.637251853942871);
  ekf_test->set_X_y(130, 3.40019154548645);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(131, -1.6371756792068481);
  ekf_test->set_X_y(132, 23.400203704833984);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(133, -1.637168526649475);
  ekf_test->set_X_y(134, 25.400203704833984);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(135, -1.6371612548828125);
  ekf_test->set_X_y(136, 27.400203704833984);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(137, -1.63715398311615);
  ekf_test->set_X_y(138, 29.400203704833984);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(139, -1.6371467113494873);
  ekf_test->set_X_y(140, 31.400203704833984);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(141, -1.6371395587921143);
  ekf_test->set_X_y(142, 33.400203704833984);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(143, 1.637324333190918);
  ekf_test->set_X_y(144, 23.400192260742188);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(145, 1.637331485748291);
  ekf_test->set_X_y(146, 25.400192260742188);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(147, 1.6373387575149536);
  ekf_test->set_X_y(148, 27.400192260742188);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(149, 1.6373460292816162);
  ekf_test->set_X_y(150, 29.400192260742188);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(151, 1.6373533010482788);
  ekf_test->set_X_y(152, 31.400192260742188);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(153, 1.6373604536056519);
  ekf_test->set_X_y(154, 33.40018844604492);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(155, -1.6371322870254517);
  ekf_test->set_X_y(156, 35.400203704833984);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(157, 1.6373677253723145);
  ekf_test->set_X_y(158, 35.40018844604492);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(159, -0.5501322746276855);
  ekf_test->set_X_y(160, 35.40019989013672);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(161, 0.5498676896095276);
  ekf_test->set_X_y(162, 35.40019226074219);
  ekf_test->push_to_colors(colors::Color::orange);
  ekf_test->set_X_y(163, -1.7835502624511719);
  ekf_test->set_X_y(164, 15.88513469696045);
  ekf_test->push_to_colors(colors::Color::large_orange);
  ekf_test->set_X_y(165, -1.7835609912872314);
  ekf_test->set_X_y(166, 12.915034294128418);
  ekf_test->push_to_colors(colors::Color::large_orange);
  ekf_test->set_X_y(167, 1.7831497192382812);
  ekf_test->set_X_y(168, 15.88512134552002);
  ekf_test->push_to_colors(colors::Color::large_orange);
  ekf_test->set_X_y(169, 1.7831389904022217);
  ekf_test->set_X_y(170, 12.915020942687988);
  ekf_test->push_to_colors(colors::Color::large_orange);
  ekf_test->set_X_y(171, -2.6371395587921143);
  ekf_test->set_X_y(172, 34.400203704833984);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(173, 4.637324333190918);
  ekf_test->set_X_y(174, 24.400192260742188);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(175, 4.637331485748291);
  ekf_test->set_X_y(176, 24.400192260742188);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(177, 5.6373387575149536);
  ekf_test->set_X_y(178, 25.400192260742188);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(179, 6.6373460292816162);
  ekf_test->set_X_y(180, 26.400192260742188);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(181, 2.6373533010482788);
  ekf_test->set_X_y(182, 32.450192260742188);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(183, 4.6353604536056519);
  ekf_test->set_X_y(184, 31.10018844604492);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(185, -1.6371322870254517);
  ekf_test->set_X_y(186, 33.300203704833984);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(187, 4.6373677253723145);
  ekf_test->set_X_y(188, 34.44018844604492);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(189, -1.5201322746276855);
  ekf_test->set_X_y(190, 35.20019989013672);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(191, 1.5498676896095276);
  ekf_test->set_X_y(192, 31.42019226074219);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(193, -3.7835502624511719);
  ekf_test->set_X_y(194, 14.88513469696045);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(195, -6.7835609912872314);
  ekf_test->set_X_y(196, 16.915034294128418);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(197, 6.7831497192382812);
  ekf_test->set_X_y(198, 16.86512134552002);
  ekf_test->push_to_colors(colors::Color::yellow);
  ekf_test->set_X_y(199, 3.7831389904022217);
  ekf_test->set_X_y(200, 16.715020942687988);
  ekf_test->push_to_colors(colors::Color::blue);
  ekf_test->set_X_y(201, 7.1831389904022217);
  ekf_test->set_X_y(202, 13.115020942687988);
  ekf_test->push_to_colors(colors::Color::yellow);
*/