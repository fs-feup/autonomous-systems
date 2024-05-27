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

  custom_interfaces::msg::ConeArray cone_array_msg;      // message to receive
  custom_interfaces::msg::PathPointArray received_path;  // message to send

  // Publisher and Subscriber
  std::shared_ptr<rclcpp::Publisher<custom_interfaces::msg::ConeArray>> map_publisher;
  std::shared_ptr<rclcpp::Publisher<custom_interfaces::msg::VehicleState>> vehicle_state_publisher_;
  std::shared_ptr<rclcpp::Subscription<custom_interfaces::msg::PathPointArray>> control_sub;

  void SetUp() override {
    rclcpp::init(0, nullptr);

    // Init Nodes
    control_receiver = rclcpp::Node::make_shared("control_receiver");  // gets path from planning
    locmap_sender = rclcpp::Node::make_shared("locmap_sender");        // publishes map from loc_map
    planning_test = std::make_shared<Planning>();                      // processes planning

    cone_array_msg = custom_interfaces::msg::ConeArray();  // init received message

    // Init Publisher
    map_publisher = locmap_sender->create_publisher<custom_interfaces::msg::ConeArray>(
        "/state_estimation/map", 10);

    vehicle_state_publisher_ =
        locmap_sender->create_publisher<custom_interfaces::msg::VehicleState>(
            "/state_estimation/vehicle_state", 10);

    // Init Subscriber
    control_sub = control_receiver->create_subscription<custom_interfaces::msg::PathPointArray>(
        "/path_planning/path", 10,
        [this](const custom_interfaces::msg::PathPointArray::SharedPtr msg) {
          RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Received path in mock control node");
          received_path = *msg;
          rclcpp::shutdown();
          RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Ended control callback in mock control");
        });
  }

  void TearDown() override {
    control_receiver.reset();
    locmap_sender.reset();
    map_publisher.reset();
    control_sub.reset();
    planning_test.reset();
    vehicle_state_publisher_.reset();
    rclcpp::shutdown();
  }

  std::chrono::duration<double, std::milli> run_nodes(
      const custom_interfaces::msg::ConeArray &track_msg,
      const custom_interfaces::msg::VehicleState &state_msg) {
    this->map_publisher->publish(track_msg);  // send the cones
    this->vehicle_state_publisher_->publish(state_msg);

    // Add nodes to be executed
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(this->locmap_sender);
    executor.add_node(this->planning_test);
    executor.add_node(this->control_receiver);

    auto start_time = std::chrono::high_resolution_clock::now();
    executor.spin();  // Execute nodes
    auto end_time = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end_time - start_time);
  }
};

TEST_F(IntegrationTest, PUBLISH_PATH1) {
  std::vector<Cone> cone_array = {
      Cone(1, 19, 2),  Cone(3, 22, 2),   Cone(5, 25, 2),   Cone(7, 28, 2),  Cone(9, 31, 2),
      Cone(11, 34, 2), Cone(13, 1, 2),   Cone(15, 4, 2),   Cone(17, 7, 2),  Cone(19, 10, 2),
      Cone(21, 13, 2), Cone(23, 16, 2),  Cone(0, 19, -2),  Cone(2, 22, -2), Cone(4, 25, -2),
      Cone(6, 28, -2), Cone(8, 31, -2),  Cone(10, 34, -2), Cone(12, 1, -2), Cone(14, 4, -2),
      Cone(16, 7, -2), Cone(18, 10, -2), Cone(20, 13, -2), Cone(22, 16, -2)};
  this->cone_array_msg = custom_interfaces_array_from_vector(cone_array);

  custom_interfaces::msg::VehicleState vehicle_state;
  vehicle_state.position.x = 0;
  vehicle_state.position.y = 0;
  vehicle_state.theta = 0;
  vehicle_state.linear_velocity = 0;
  vehicle_state.angular_velocity = 0;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing cone array with size: %ld",
               cone_array_msg.cone_array.size());

  auto duration = run_nodes(cone_array_msg, vehicle_state);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Execution time: %f ms", duration.count());
  for (const auto &p : received_path.pathpoint_array) {
    EXPECT_DOUBLE_EQ(p.y, 0);
    EXPECT_LE(p.x, 35);
    EXPECT_GE(p.x, -0.5);
  }
  EXPECT_EQ(static_cast<long unsigned>(received_path.pathpoint_array.size()),
            (long unsigned int)241);
}

TEST_F(IntegrationTest, PUBLISH_PATH2) {
  std::vector<Cone> cone_array = {
      Cone(1, (float)1.414, (float)-1.414),   Cone(3, (float)3.184, (float)0.356),
      Cone(5, (float)4.954, (float)2.126),    Cone(7, (float)6.724, (float)3.896),
      Cone(9, (float)8.494, (float)5.666),    Cone(11, (float)10.264, (float)7.436),
      Cone(13, (float)12.034, (float)9.206),  Cone(15, (float)13.804, (float)10.976),
      Cone(17, (float)15.574, (float)12.746), Cone(19, (float)17.344, (float)14.516),
      Cone(21, (float)19.114, (float)16.286), Cone(23, (float)20.884, (float)18.056),
      Cone(0, (float)-1.414, (float)1.414),   Cone(2, (float)0.356, (float)3.184),
      Cone(4, (float)2.126, (float)4.954),    Cone(6, (float)3.896, (float)6.724),
      Cone(8, (float)5.666, (float)8.494),    Cone(10, (float)7.436, (float)10.264),
      Cone(12, (float)9.206, (float)12.034),  Cone(14, (float)10.976, (float)13.804),
      Cone(16, (float)12.746, (float)15.574), Cone(18, (float)14.516, (float)17.344),
      Cone(20, (float)16.286, (float)19.114), Cone(22, (float)18.056, (float)20.884)};
  this->cone_array_msg = custom_interfaces_array_from_vector(cone_array);

  custom_interfaces::msg::VehicleState vehicle_state;
  vehicle_state.position.x = 0;
  vehicle_state.position.y = 0;
  vehicle_state.theta = 0.785398;  // 45 degrees
  vehicle_state.linear_velocity = 0;
  vehicle_state.angular_velocity = 0;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing cone array with size: %ld",
               cone_array_msg.cone_array.size());
  auto duration = run_nodes(cone_array_msg, vehicle_state);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Execution time: %f ms", duration.count());
  for (const auto &p : received_path.pathpoint_array) {
    EXPECT_LE(p.y - p.x, 0.1);
  }
  EXPECT_EQ(static_cast<long unsigned>(received_path.pathpoint_array.size()),
            (long unsigned int)241);
}

TEST_F(IntegrationTest, PUBLISH_PATH3) {
  custom_interfaces::msg::Cone cone_to_send;

  std::vector<Cone> cone_array = {
      Cone(0, (float)-1.414, (float)-1.414),   Cone(2, (float)-3.184, (float)0.356),
      Cone(4, (float)-4.954, (float)2.126),    Cone(6, (float)-6.724, (float)3.896),
      Cone(8, (float)-8.494, (float)5.666),    Cone(10, (float)-10.264, (float)7.436),
      Cone(12, (float)-12.034, (float)9.206),  Cone(14, (float)-13.804, (float)10.976),
      Cone(16, (float)-15.574, (float)12.746), Cone(18, (float)-17.344, (float)14.516),
      Cone(20, (float)-19.114, (float)16.286), Cone(22, (float)-20.884, (float)18.056),
      Cone(1, (float)1.414, (float)1.414),     Cone(3, (float)-0.356, (float)3.184),
      Cone(5, (float)-2.126, (float)4.954),    Cone(7, (float)-3.896, (float)6.724),
      Cone(9, (float)-5.666, (float)8.494),    Cone(11, (float)-7.436, (float)10.264),
      Cone(13, (float)-9.206, (float)12.034),  Cone(15, (float)-10.976, (float)13.804),
      Cone(17, (float)-12.746, (float)15.574), Cone(19, (float)-14.516, (float)17.344),
      Cone(21, (float)-16.286, (float)19.114), Cone(23, (float)-18.056, (float)20.884)};
  this->cone_array_msg = custom_interfaces_array_from_vector(cone_array);

  custom_interfaces::msg::VehicleState vehicle_state;
  vehicle_state.position.x = 0;
  vehicle_state.position.y = 0;
  vehicle_state.theta = 2.3561945;  // 135 degrees
  vehicle_state.linear_velocity = 0;
  vehicle_state.angular_velocity = 0;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing cone array with size: %ld",
               cone_array_msg.cone_array.size());
  auto duration = run_nodes(cone_array_msg, vehicle_state);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Execution time: %f ms", duration.count());
  for (const auto &p : received_path.pathpoint_array) {
    EXPECT_LE(p.y + p.x, 0.1);
  }
  EXPECT_EQ(static_cast<long unsigned>(received_path.pathpoint_array.size()),
            (long unsigned int)241);
}

TEST_F(IntegrationTest, PUBLISH_PATH4) {
  std::vector<Cone> cone_array = {
      Cone(0, (float)-1.414, (float)1.414),     Cone(2, (float)-3.184, (float)-0.356),
      Cone(4, (float)-4.954, (float)-2.126),    Cone(6, (float)-6.724, (float)-3.896),
      Cone(8, (float)-8.494, (float)-5.666),    Cone(10, (float)-10.264, (float)-7.436),
      Cone(12, (float)-12.034, (float)-9.206),  Cone(14, (float)-13.804, (float)-10.976),
      Cone(16, (float)-15.574, (float)-12.746), Cone(18, (float)-17.344, (float)-14.516),
      Cone(20, (float)-19.114, (float)-16.286), Cone(22, (float)-20.884, (float)-18.056),
      Cone(1, (float)1.414, (float)-1.414),     Cone(3, (float)-0.356, (float)-3.184),
      Cone(5, (float)-2.126, (float)-4.954),    Cone(7, (float)-3.896, (float)-6.724),
      Cone(9, (float)-5.666, (float)-8.494),    Cone(11, (float)-7.436, (float)-10.264),
      Cone(13, (float)-9.206, (float)-12.034),  Cone(15, (float)-10.976, (float)-13.804),
      Cone(17, (float)-12.746, (float)-15.574), Cone(19, (float)-14.516, (float)-17.344),
      Cone(21, (float)-16.286, (float)-19.114), Cone(23, (float)-18.056, (float)-20.884)};
  this->cone_array_msg = custom_interfaces_array_from_vector(cone_array);

  custom_interfaces::msg::VehicleState vehicle_state;
  vehicle_state.position.x = 0;
  vehicle_state.position.y = 0;
  vehicle_state.theta = 3.92699;  // 225 degrees
  vehicle_state.linear_velocity = 0;
  vehicle_state.angular_velocity = 0;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing cone array with size: %ld",
               cone_array_msg.cone_array.size());
  auto duration = run_nodes(cone_array_msg, vehicle_state);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Execution time: %f ms", duration.count());
  for (const auto &p : received_path.pathpoint_array) {
    EXPECT_LE(p.y - p.x, 0.1);
  }
  EXPECT_EQ(static_cast<long unsigned>(received_path.pathpoint_array.size()),
            (long unsigned int)241);
}

TEST_F(IntegrationTest, PUBLISH_PATH5) {
  std::vector<Cone> cone_array = {
      Cone(0, (float)1.414, (float)1.414),     Cone(2, (float)3.184, (float)-0.356),
      Cone(4, (float)4.954, (float)-2.126),    Cone(6, (float)6.724, (float)-3.896),
      Cone(8, (float)8.494, (float)-5.666),    Cone(10, (float)10.264, (float)-7.436),
      Cone(12, (float)12.034, (float)-9.206),  Cone(14, (float)13.804, (float)-10.976),
      Cone(16, (float)15.574, (float)-12.746), Cone(18, (float)17.344, (float)-14.516),
      Cone(20, (float)19.114, (float)-16.286), Cone(22, (float)20.884, (float)-18.056),
      Cone(1, (float)-1.414, (float)-1.414),   Cone(3, (float)0.356, (float)-3.184),
      Cone(5, (float)2.126, (float)-4.954),    Cone(7, (float)3.896, (float)-6.724),
      Cone(9, (float)5.666, (float)-8.494),    Cone(11, (float)7.436, (float)-10.264),
      Cone(13, (float)9.206, (float)-12.034),  Cone(15, (float)10.976, (float)-13.804),
      Cone(17, (float)12.746, (float)-15.574), Cone(19, (float)14.516, (float)-17.344),
      Cone(21, (float)16.286, (float)-19.114), Cone(23, (float)18.056, (float)-20.884)};
  this->cone_array_msg = custom_interfaces_array_from_vector(cone_array);

  custom_interfaces::msg::VehicleState vehicle_state;
  vehicle_state.position.x = 0;
  vehicle_state.position.y = 0;
  vehicle_state.theta = 5.49779;  // 315 degrees
  vehicle_state.linear_velocity = 0;
  vehicle_state.angular_velocity = 0;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing cone array with size: %ld",
               cone_array_msg.cone_array.size());
  auto duration = run_nodes(cone_array_msg, vehicle_state);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Execution time: %f ms", duration.count());
  for (const auto &p : received_path.pathpoint_array) {
    EXPECT_LE(p.y + p.x, 0.1);
  }
  EXPECT_EQ(static_cast<long unsigned>(received_path.pathpoint_array.size()),
            (long unsigned int)241);
}

TEST_F(IntegrationTest, PUBLISH_PATH6) {
  std::vector<Cone> cone_array = {
      Cone(1, 2, 19),   Cone(3, 2, 22),   Cone(5, 2, 25),   Cone(7, 2, 28),   Cone(9, 2, 31),
      Cone(11, 2, 34),  Cone(13, 2, 1),   Cone(15, 2, 4),   Cone(17, 2, 7),   Cone(19, 2, 10),
      Cone(21, 2, 13),  Cone(23, 2, 16),  Cone(0, -2, 1),   Cone(2, -2, 4),   Cone(4, -2, 7),
      Cone(6, -2, 25),  Cone(8, -2, 28),  Cone(10, -2, 31), Cone(12, -2, 34), Cone(14, -2, 10),
      Cone(16, -2, 13), Cone(18, -2, 16), Cone(20, -2, 19), Cone(22, -2, 22)};
  this->cone_array_msg = custom_interfaces_array_from_vector(cone_array);

  custom_interfaces::msg::VehicleState vehicle_state;
  vehicle_state.position.x = 0;
  vehicle_state.position.y = 0;
  vehicle_state.theta = 1.5708;  // 90 degrees
  vehicle_state.linear_velocity = 0;
  vehicle_state.angular_velocity = 0;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing cone array with size: %ld",
               cone_array_msg.cone_array.size());
  auto duration = run_nodes(cone_array_msg, vehicle_state);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Execution time: %f ms", duration.count());
  for (const auto &p : received_path.pathpoint_array) {
    EXPECT_LE(fabs(p.x), 0.1);
    EXPECT_LE(p.y, 35);
    EXPECT_GE(p.y, -1);
  }
  EXPECT_EQ(static_cast<long unsigned>(received_path.pathpoint_array.size()),
            (long unsigned int)241);
}

// empty track
TEST_F(IntegrationTest, PUBLISH_PATH7) {
  custom_interfaces::msg::VehicleState vehicle_state;
  vehicle_state.position.x = 0;
  vehicle_state.position.y = 0;
  vehicle_state.theta = 1.5708;  // 90 degrees
  vehicle_state.linear_velocity = 0;
  vehicle_state.angular_velocity = 0;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing cone array with size: %ld",
               cone_array_msg.cone_array.size());

  auto duration = run_nodes(this->cone_array_msg, vehicle_state);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Execution time: %f ms", duration.count());
  EXPECT_EQ(static_cast<long unsigned>(received_path.pathpoint_array.size()), (long unsigned int)0);
}

TEST_F(IntegrationTest, PUBLISH_PATH8) {
  custom_interfaces::msg::Cone cone_to_send;
  // Yellow Cones
  cone_to_send.position.x = 2;
  cone_to_send.position.y = 19;
  cone_to_send.color = "yellow_cone";
  cone_array_msg.cone_array.push_back(cone_to_send);

  custom_interfaces::msg::VehicleState vehicle_state;
  vehicle_state.position.x = 0;
  vehicle_state.position.y = 0;
  vehicle_state.theta = 1.5708;  // 90 degrees
  vehicle_state.linear_velocity = 0;
  vehicle_state.angular_velocity = 0;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing cone array with size: %ld",
               cone_array_msg.cone_array.size());
  auto duration = run_nodes(cone_array_msg, vehicle_state);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Execution time: %f ms", duration.count());
  EXPECT_EQ(static_cast<long unsigned>(received_path.pathpoint_array.size()), (long unsigned int)0);
}
