#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <custom_interfaces/msg/cone_array.hpp>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "perception/parameters_factory.hpp"
#include "perception/perception_node.hpp"

class PerceptionIntegrationTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr cones_subscriber_;
  custom_interfaces::msg::ConeArray::SharedPtr cones_result_;
  bool cones_received_;

  void SetUp() override {
    rclcpp::init(0, nullptr);

    // Create a node for publishing and subscribing
    test_node_ = std::make_shared<rclcpp::Node>("test_node");

    // Publisher for input point cloud
    pcl_publisher_ = test_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_points",
                                                                                 rclcpp::QoS(10));

    // Subscriber for cone array
    cones_subscriber_ = test_node_->create_subscription<custom_interfaces::msg::ConeArray>(
        "/perception/cones", 1, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
          cones_result_ = msg;
          cones_received_ = true;
        });

    cones_received_ = false;
  }

  bool inRange(double value, double target, double tolerance) {
    return value >= target - tolerance && value <= target + tolerance;
  };

  int checkPosition(custom_interfaces::msg::ConeArray::SharedPtr cones_result, double xTarget,
                    double yTarget, double tolerance, bool large) {
    int count = 0;
    for (auto cone : cones_result->cone_array) {
      if (inRange(cone.position.x, xTarget, tolerance) &&
          inRange(cone.position.y, yTarget, tolerance) && cone.is_large == large) {
        count++;
      }
    }
    return count;
  };

  void TearDown() override { rclcpp::shutdown(); }

  void publish_pcd(const std::string& pcd_file_path) {
    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file_path, pcl_cloud) == -1) {
      throw std::runtime_error("Could not load PCD file: " + pcd_file_path);
    }

    // Convert PCL cloud to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pcl_cloud, msg);
    msg.header.frame_id = "hesai_lidar";
    msg.header.stamp = test_node_->now();

    // Publish the message
    pcl_publisher_->publish(msg);
  }
};

TEST_F(PerceptionIntegrationTest, StraigthLine) {
  auto params = load_adapter_parameters();
  rclcpp::Node::SharedPtr perception_node = std::make_shared<Perception>(params);
  ASSERT_NE(perception_node, nullptr) << "Failed to initialize Perception node.";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(test_node_);

  std::string pcd_file_path = "/home/ws/src/perception/test/point_clouds/straigth_line.pcd";
  ASSERT_TRUE(std::filesystem::exists(pcd_file_path))
      << "PCD file does not exist: " << pcd_file_path;

  try {
    publish_pcd(pcd_file_path);
  } catch (const std::exception& e) {
    FAIL() << "Failed to publish PCD: " << e.what();
  }

  auto start_time = std::chrono::steady_clock::now();
  while (!cones_received_ &&
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(100));  // Prevent busy-waiting
  }

  EXPECT_TRUE(cones_received_) << "No cones received within the timeout.";
  if (cones_received_) {
    EXPECT_EQ(cones_result_->cone_array.size(), 6)
        << "Wrong number of cones detected in the point cloud.";

    EXPECT_EQ(checkPosition(cones_result_, 3.2, 1.55, 0.2, false), 1)
        << "Wrong detection in the 1st cone on the left";
    EXPECT_EQ(checkPosition(cones_result_, 3.2, -1.8, 0.2, false), 1)
        << "Wrong detection in the 1st cone on the right";
    EXPECT_EQ(checkPosition(cones_result_, 5.94, 1.55, 0.2, false), 1)
        << "Wrong detection in the 2nd cone on the left";
    EXPECT_EQ(checkPosition(cones_result_, 5.94, -1.8, 0.2, false), 1)
        << "Wrong detection in the 2nd cone on the right";
    EXPECT_EQ(checkPosition(cones_result_, 8.8, 1.55, 0.2, false), 1)
        << "Wrong detection in the 3rd cone on the left";
    EXPECT_EQ(checkPosition(cones_result_, 8.8, -1.8, 0.2, false), 1)
        << "Wrong detection in the 3rd cone on the right";
  }

  executor.cancel();
}

TEST_F(PerceptionIntegrationTest, Accelaration_close) {
  auto params = load_adapter_parameters();
  rclcpp::Node::SharedPtr perception_node = std::make_shared<Perception>(params);
  ASSERT_NE(perception_node, nullptr) << "Failed to initialize Perception node.";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(test_node_);

  std::string pcd_file_path =
      "/home/ws/src/perception/test/point_clouds/accelaration_end_close.pcd";
  ASSERT_TRUE(std::filesystem::exists(pcd_file_path))
      << "PCD file does not exist: " << pcd_file_path;

  try {
    publish_pcd(pcd_file_path);
  } catch (const std::exception& e) {
    FAIL() << "Failed to publish PCD: " << e.what();
  }

  auto start_time = std::chrono::steady_clock::now();
  while (!cones_received_ &&
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(100));  // Prevent busy-waiting
  }

  EXPECT_TRUE(cones_received_) << "No cones received within the timeout.";
  if (cones_received_) {
    EXPECT_EQ(cones_result_->cone_array.size(), 12)
        << "Wrong number of cones detected in the point cloud.";
  }

  executor.cancel();
}