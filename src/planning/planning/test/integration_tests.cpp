#include "gtest/gtest.h"
#include "planning/planning.hpp"
#include "rclcpp/rclcpp.hpp"

class IntegrationTest : public::testing::Test {
 protected:
  
  std::shared_ptr<rclcpp::Node> locmap_sender;
  std::shared_ptr<rclcpp::Node> control_receiver;
  std::shared_ptr<rclcpp::Node> planning_test;

  std::shared_ptr<custom_interfaces::msg::ConeArray> cone_array_msg;
  custom_interfaces::msg::PointArray received_path;

  std::shared_ptr<rclcpp::Publisher<custom_interfaces::msg::ConeArray>> map_publisher;

  std::shared_ptr<rclcpp::Subscription<custom_interfaces::msg::PointArray>> control_sub;


    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        control_receiver = rclcpp::Node::make_shared(
        "control_receiver");  // test node, receives path from planning
        locmap_sender = rclcpp::Node::make_shared("locmap_sender"); // test node, publishes map from loc_map

        cone_array_msg = std::make_shared<custom_interfaces::msg::ConeArray>();

        map_publisher = locmap_sender->create_publisher<custom_interfaces::msg::ConeArray>(
            "track_map", 10);

        control_sub = control_receiver->create_subscription<custom_interfaces::msg::PointArray>(
            "planning_local", 10, [this](const custom_interfaces::msg::PointArray::SharedPtr msg) {

            received_path = *msg;
            rclcpp::shutdown();
            });  // subscribe to planning topic to get the published path

        planning_test = std::make_shared<Planning>();
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
    cone_to_send.position.x = -1;
    cone_to_send.position.y = 0;
    cone_to_send.color = "yellow_cone";
    cone_array_msg->cone_array.push_back(cone_to_send);

    cone_to_send.position.x = -1;
    cone_to_send.position.y = 1;
    cone_to_send.color = "yellow_cone";
    cone_array_msg->cone_array.push_back(cone_to_send);

    cone_to_send.position.x = 1;
    cone_to_send.position.y = 0;
    cone_to_send.color = "blue_cone";
    cone_array_msg->cone_array.push_back(cone_to_send);

    cone_to_send.position.x = 1;
    cone_to_send.position.y = 1;
    cone_to_send.color = "blue_cone";
    cone_array_msg->cone_array.push_back(cone_to_send);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// AFTER ADDING FORTH: %ld ///////\n",
                cone_array_msg->cone_array.size());
    // complete loc map node
    std::this_thread::sleep_for(std::chrono::seconds(3));
    map_publisher->publish(*cone_array_msg);          // send the cones
    rclcpp::executors::MultiThreadedExecutor executor;  // create executor and add all nodes
    executor.add_node(locmap_sender);
    executor.add_node(planning_test);
    executor.add_node(control_receiver);
    auto start_time = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\nPre spin\n");
    executor.spin();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\nAfter spin\n");
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);


    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());


    std::ofstream file("../../src/planning/test/integration_test.csv", std::ios::app);  
    file << "planning, all, 4 cones, " << duration.count() << "\n";
    file.close();

    EXPECT_GE(received_path.points.size(), (long unsigned int)3);
}