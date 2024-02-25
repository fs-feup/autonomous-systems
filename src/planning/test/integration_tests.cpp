#include <fstream>

#include "gtest/gtest.h"
#include "planning/planning.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/files.hpp"

class IntegrationTest : public ::testing::Test {
 protected:
  // Required Nodes
  std::shared_ptr<rclcpp::Node> locmap_sender;
  std::shared_ptr<rclcpp::Node> control_receiver;
  std::shared_ptr<rclcpp::Node> planning_test;

  custom_interfaces::msg::ConeArray cone_array_msg;  // message to receive
  custom_interfaces::msg::PathPointArray received_path;  // message to send

  // Publisher and Subscriber
  std::shared_ptr<rclcpp::Publisher<custom_interfaces::msg::ConeArray>> map_publisher;
  std::shared_ptr<rclcpp::Subscription<custom_interfaces::msg::PathPointArray>> control_sub;

  void SetUp() override {
    rclcpp::init(0, nullptr);

    // Init Nodes
    control_receiver = rclcpp::Node::make_shared("control_receiver");  // gets path from planning
    locmap_sender = rclcpp::Node::make_shared("locmap_sender");        // publishes map from loc_map
    planning_test = std::make_shared<Planning>();                      // processes planning

    cone_array_msg = custom_interfaces::msg::ConeArray();  // init received message

    // Init Publisher
    map_publisher =
        locmap_sender->create_publisher<custom_interfaces::msg::ConeArray>("track_map", 10);

    // Init Subscriber
    control_sub = control_receiver->create_subscription<custom_interfaces::msg::PathPointArray>(
        "planning_local", 10, [this](const custom_interfaces::msg::PathPointArray::SharedPtr msg) {
          received_path = *msg;
          rclcpp::shutdown();  // When receives message shuts down
        });
  }

  void TearDown() override {
    control_receiver.reset();
    locmap_sender.reset();
    map_publisher.reset();
    control_sub.reset();
    planning_test.reset();

    rclcpp::shutdown();
  }
};

TEST_F(IntegrationTest, PUBLISH_PATH1) {
  custom_interfaces::msg::Cone cone_to_send;
  // Yellow Cones

  cone_to_send.position.x = 1;
  cone_to_send.position.y = 1;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 2;
  cone_to_send.position.y = 1;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 3;
  cone_to_send.position.y = 1;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 4;
  cone_to_send.position.y = 1;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 4.7;
  cone_to_send.position.y = 1.2;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 5.2;
  cone_to_send.position.y = 1.5;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 5.5;
  cone_to_send.position.y = 1.9;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 5.7;
  cone_to_send.position.y = 2.7;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 5.8;
  cone_to_send.position.y = 3.8;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 6.0;
  cone_to_send.position.y = 4.5;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 6.5;
  cone_to_send.position.y = 5.1;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 7.0;
  cone_to_send.position.y = 5.5;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  // Blue Cones

  cone_to_send.position.x = 1;
  cone_to_send.position.y = -1;
  cone_to_send.color = "blue_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 2;
  cone_to_send.position.y = -1;
  cone_to_send.color = "blue_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 3;
  cone_to_send.position.y = -1;
  cone_to_send.color = "blue_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 4;
  cone_to_send.position.y = -1;
  cone_to_send.color = "blue_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 4.7;
  cone_to_send.position.y = -0.8;
  cone_to_send.color = "blue_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 5.2;
  cone_to_send.position.y = -0.5;
  cone_to_send.color = "blue_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 5.5;
  cone_to_send.position.y = -0.1;
  cone_to_send.color = "blue_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 5.7;
  cone_to_send.position.y = 0.7;
  cone_to_send.color = "blue_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 5.8;
  cone_to_send.position.y = 1.8;
  cone_to_send.color = "blue_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 6.0;
  cone_to_send.position.y = 2.5;
  cone_to_send.color = "blue_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 6.5;
  cone_to_send.position.y = 3.1;
  cone_to_send.color = "blue_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  cone_to_send.position.x = 7.0;
  cone_to_send.position.y = 3.5;
  cone_to_send.color = "blue_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing cone array with size: %ld",
              cone_array_msg.cone_array.size());

  // std::this_thread::sleep_for(std::chrono::seconds(3));
  map_publisher->publish(cone_array_msg);  // send the cones

  // Add nodes to be executed
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(locmap_sender);
  executor.add_node(planning_test);
  executor.add_node(control_receiver);

  auto start_time = std::chrono::high_resolution_clock::now();
  executor.spin();  // Execute nodes
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %f ms", duration.count());
  std::ofstream file = openWriteFile("src/performance/exec_time/planning.csv");
  file << "planning, all, 4 cones, " << duration.count() << "\n";
  file.close();
  EXPECT_EQ(static_cast<long unsigned>(
  received_path.pathpoint_array.size()), (long unsigned int)21);
}