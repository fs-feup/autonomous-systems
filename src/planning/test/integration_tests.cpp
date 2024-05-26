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
  std::vector<Cone *> cone_array = {
      new Cone(1, 19, 2),  new Cone(3, 22, 2),   new Cone(5, 25, 2),   new Cone(7, 28, 2),
      new Cone(9, 31, 2),  new Cone(11, 34, 2),  new Cone(13, 1, 2),   new Cone(15, 4, 2),
      new Cone(17, 7, 2),  new Cone(19, 10, 2),  new Cone(21, 13, 2),  new Cone(23, 16, 2),
      new Cone(0, 19, -2), new Cone(2, 22, -2),  new Cone(4, 25, -2),  new Cone(6, 28, -2),
      new Cone(8, 31, -2), new Cone(10, 34, -2), new Cone(12, 1, -2),  new Cone(14, 4, -2),
      new Cone(16, 7, -2), new Cone(18, 10, -2), new Cone(20, 13, -2), new Cone(22, 16, -2)};
  this->cone_array_msg = custom_interfaces_array_from_vector(cone_array);

  for (Cone *c : cone_array) {
    delete c;
  }  // send the cones

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
  std::vector<Cone *> cone_array = {
      new Cone(1, 1.414, -1.414),   new Cone(3, 3.184, 0.356),    new Cone(5, 4.954, 2.126),
      new Cone(7, 6.724, 3.896),    new Cone(9, 8.494, 5.666),    new Cone(11, 10.264, 7.436),
      new Cone(13, 12.034, 9.206),  new Cone(15, 13.804, 10.976), new Cone(17, 15.574, 12.746),
      new Cone(19, 17.344, 14.516), new Cone(21, 19.114, 16.286), new Cone(23, 20.884, 18.056),
      new Cone(0, -1.414, 1.414),   new Cone(2, 0.356, 3.184),    new Cone(4, 2.126, 4.954),
      new Cone(6, 3.896, 6.724),    new Cone(8, 5.666, 8.494),    new Cone(10, 7.436, 10.264),
      new Cone(12, 9.206, 12.034),  new Cone(14, 10.976, 13.804), new Cone(16, 12.746, 15.574),
      new Cone(18, 14.516, 17.344), new Cone(20, 16.286, 19.114), new Cone(22, 18.056, 20.884)};
  this->cone_array_msg = custom_interfaces_array_from_vector(cone_array);

  for (Cone *c : cone_array) {
    delete c;
  }

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

  std::vector<Cone *> cone_array = {
      new Cone(0, -1.414, -1.414),   new Cone(2, -3.184, 0.356),    new Cone(4, -4.954, 2.126),
      new Cone(6, -6.724, 3.896),    new Cone(8, -8.494, 5.666),    new Cone(10, -10.264, 7.436),
      new Cone(12, -12.034, 9.206),  new Cone(14, -13.804, 10.976), new Cone(16, -15.574, 12.746),
      new Cone(18, -17.344, 14.516), new Cone(20, -19.114, 16.286), new Cone(22, -20.884, 18.056),
      new Cone(1, 1.414, 1.414),     new Cone(3, -0.356, 3.184),    new Cone(5, -2.126, 4.954),
      new Cone(7, -3.896, 6.724),    new Cone(9, -5.666, 8.494),    new Cone(11, -7.436, 10.264),
      new Cone(13, -9.206, 12.034),  new Cone(15, -10.976, 13.804), new Cone(17, -12.746, 15.574),
      new Cone(19, -14.516, 17.344), new Cone(21, -16.286, 19.114), new Cone(23, -18.056, 20.884)};
  this->cone_array_msg = custom_interfaces_array_from_vector(cone_array);

  for (Cone *c : cone_array) {
    delete c;
  }

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
  std::vector<Cone *> cone_array = {new Cone(0, -1.414, 1.414),     new Cone(2, -3.184, -0.356),
                                    new Cone(4, -4.954, -2.126),    new Cone(6, -6.724, -3.896),
                                    new Cone(8, -8.494, -5.666),    new Cone(10, -10.264, -7.436),
                                    new Cone(12, -12.034, -9.206),  new Cone(14, -13.804, -10.976),
                                    new Cone(16, -15.574, -12.746), new Cone(18, -17.344, -14.516),
                                    new Cone(20, -19.114, -16.286), new Cone(22, -20.884, -18.056),
                                    new Cone(1, 1.414, -1.414),     new Cone(3, -0.356, -3.184),
                                    new Cone(5, -2.126, -4.954),    new Cone(7, -3.896, -6.724),
                                    new Cone(9, -5.666, -8.494),    new Cone(11, -7.436, -10.264),
                                    new Cone(13, -9.206, -12.034),  new Cone(15, -10.976, -13.804),
                                    new Cone(17, -12.746, -15.574), new Cone(19, -14.516, -17.344),
                                    new Cone(21, -16.286, -19.114), new Cone(23, -18.056, -20.884)};
  this->cone_array_msg = custom_interfaces_array_from_vector(cone_array);

  for (Cone *c : cone_array) {
    delete c;
  }

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
  std::vector<Cone *> cone_array = {
      new Cone(0, 1.414, 1.414),     new Cone(2, 3.184, -0.356),    new Cone(4, 4.954, -2.126),
      new Cone(6, 6.724, -3.896),    new Cone(8, 8.494, -5.666),    new Cone(10, 10.264, -7.436),
      new Cone(12, 12.034, -9.206),  new Cone(14, 13.804, -10.976), new Cone(16, 15.574, -12.746),
      new Cone(18, 17.344, -14.516), new Cone(20, 19.114, -16.286), new Cone(22, 20.884, -18.056),
      new Cone(1, -1.414, -1.414),   new Cone(3, 0.356, -3.184),    new Cone(5, 2.126, -4.954),
      new Cone(7, 3.896, -6.724),    new Cone(9, 5.666, -8.494),    new Cone(11, 7.436, -10.264),
      new Cone(13, 9.206, -12.034),  new Cone(15, 10.976, -13.804), new Cone(17, 12.746, -15.574),
      new Cone(19, 14.516, -17.344), new Cone(21, 16.286, -19.114), new Cone(23, 18.056, -20.884)};
  this->cone_array_msg = custom_interfaces_array_from_vector(cone_array);

  for (Cone *c : cone_array) {
    delete c;
  }

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
  std::vector<Cone *> cone_array = {
      new Cone(1, 2, 19),   new Cone(3, 2, 22),   new Cone(5, 2, 25),   new Cone(7, 2, 28),
      new Cone(9, 2, 31),   new Cone(11, 2, 34),  new Cone(13, 2, 1),   new Cone(15, 2, 4),
      new Cone(17, 2, 7),   new Cone(19, 2, 10),  new Cone(21, 2, 13),  new Cone(23, 2, 16),
      new Cone(0, -2, 1),   new Cone(2, -2, 4),   new Cone(4, -2, 7),   new Cone(6, -2, 25),
      new Cone(8, -2, 28),  new Cone(10, -2, 31), new Cone(12, -2, 34), new Cone(14, -2, 10),
      new Cone(16, -2, 13), new Cone(18, -2, 16), new Cone(20, -2, 19), new Cone(22, -2, 22)};
  this->cone_array_msg = custom_interfaces_array_from_vector(cone_array);

  for (Cone *c : cone_array) {
    delete c;
  }

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
