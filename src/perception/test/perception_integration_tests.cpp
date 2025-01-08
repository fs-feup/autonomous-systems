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

  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr cones_subscriber_;

  custom_interfaces::msg::ConeArray::SharedPtr cones_result_;
  bool cones_received_;
};

TEST_F(PerceptionIntegrationTest, AccelarationEndFar) {
  auto perception_node = std::make_shared<Perception>(load_adapter_parameters());
  ASSERT_NE(perception_node, nullptr) << "Failed to initialize Perception node.";

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(perception_node);
  executor->add_node(test_node_);

  std::thread executor_thread([&]() { executor->spin(); });

  std::string pcd_file_path = "/home/ws/src/perception/test/point_clouds/accelaration_end_far.pcd";
  ASSERT_TRUE(std::filesystem::exists(pcd_file_path))
      << "PCD file does not exist: " << pcd_file_path;

  try {
    publish_pcd(pcd_file_path);
  } catch (const std::exception& e) {
    FAIL() << "Failed to publish PCD: " << e.what();
  }

  auto start_time = std::chrono::steady_clock::now();
  while (!cones_received_ &&
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
    rclcpp::spin_some(test_node_);
  }

  EXPECT_TRUE(cones_received_) << "No cones received within the timeout.";
  if (cones_received_) {
    EXPECT_GT(cones_result_->cone_array.size(), 0) << "No cones detected in the point cloud.";
  }

  executor->cancel();
  executor_thread.join();
}
