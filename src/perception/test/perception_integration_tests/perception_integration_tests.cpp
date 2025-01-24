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

/**
 * @brief Test class for blackbox perception integration tests.
 * (When using pcd-viewer or other extension the result clouds only update after they are manually
 * reset)
 */
class PerceptionIntegrationTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr test_node_;  /// Test node created to publish a pcl and subsribe to the
                                       /// cones array on the perception node.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr cones_subscriber_;
  custom_interfaces::msg::ConeArray::SharedPtr
      cones_result_;  /// Recieves and stores perception node output
  pcl::PointCloud<pcl::PointXYZI>::Ptr
      result_cloud_;  /// Cloud that will be filled with the found cones as cylinders and saved in
                      /// the results file for visualization
  bool cones_received_;  /// Flag that determines if the cones were recieved on the test node side

  /**
   * @brief Setup publishers, subscribers and helper functions.
   */
  void SetUp() override {
    rclcpp::init(0, nullptr);

    test_node_ = std::make_shared<rclcpp::Node>("test_node");

    // Publisher for input point cloud
    pcl_publisher_ = test_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_points",
                                                                                 rclcpp::QoS(10));

    // Subscriber for output cones
    cones_subscriber_ = test_node_->create_subscription<custom_interfaces::msg::ConeArray>(
        "/perception/cones", 1, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
          cones_result_ = msg;
          cones_received_ = true;
        });

    cones_received_ = false;
    result_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  /**
   * @brief Generates a cylinder made of pcl points, used to add the found cones in the results pcl.
   */
  void generateCylinder(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float radius, float height,
                        float centerX, float centerY, int numSlices, int numHeightSegments,
                        float intensity) {
    for (int i = 0; i <= numHeightSegments; ++i) {
      float z = i * height / numHeightSegments;
      for (int j = 0; j < numSlices; ++j) {
        float theta = 2.0 * M_PI * j / numSlices;
        float x = centerX + radius * cos(theta);
        float y = centerY + radius * sin(theta);
        cloud->points.emplace_back(x, y, z, intensity);
      }
    }
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
};

/**
 * @brief Straight line test for perception node from rosbag: Accelaration_Testing_DV_1B.mcap
 */
TEST_F(PerceptionIntegrationTest, StraigthLine) {
  auto params = load_adapter_parameters();
  rclcpp::Node::SharedPtr perception_node = std::make_shared<Perception>(params);
  ASSERT_NE(perception_node, nullptr) << "Failed to initialize Perception node.";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(test_node_);

  std::string pcd_file_path =
      "../../src/perception/test/perception_integration_tests/point_clouds/straigth_line.pcd";
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
    if (cones_result_->cone_array.size() != 6)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%ld expected->%d",
                  cones_result_->cone_array.size(), 6);

    for (auto cone : cones_result_->cone_array) {
      if (cone.is_large)
        generateCylinder(result_cloud_, 0.220, 0.605, cone.position.x, cone.position.y, 100, 100,
                         100);
      else
        generateCylinder(result_cloud_, 0.150, 0.325, cone.position.x, cone.position.y, 100, 100,
                         0);
    }

    // Number of points in the PCD should be height*width
    result_cloud_->width = result_cloud_->points.size();
    result_cloud_->height = 1;

    // Save the generated cloud to a PCD file
    std::string output_pcd_path =
        "../../src/perception/test/perception_integration_tests/results/straight_line.pcd";
    if (pcl::io::savePCDFile(output_pcd_path, *result_cloud_) == -1) {
      RCLCPP_ERROR(test_node_->get_logger(), "Failed to save result PCD file.");
    } else {
      RCLCPP_INFO(test_node_->get_logger(), "Result cloud saved to: %s", output_pcd_path.c_str());
    }
  }

  executor.cancel();
}

/**
 * @brief Close to accelaration end test for perception node from rosbag:
 * Accelaration_Testing_Manual-4.mcap
 */
TEST_F(PerceptionIntegrationTest, AccelarationClose) {
  auto params = load_adapter_parameters();
  rclcpp::Node::SharedPtr perception_node = std::make_shared<Perception>(params);
  ASSERT_NE(perception_node, nullptr) << "Failed to initialize Perception node.";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(test_node_);

  std::string pcd_file_path =
      "../../src/perception/test/perception_integration_tests/point_clouds/"
      "accelaration_end_close.pcd";
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
    if (cones_result_->cone_array.size() != 12)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%ld expected->%d",
                  cones_result_->cone_array.size(), 12);

    int large_count = 0;
    for (auto cone : cones_result_->cone_array) {
      if (cone.is_large) {
        large_count++;
        generateCylinder(result_cloud_, 0.220, 0.605, cone.position.x, cone.position.y, 100, 100,
                         100);
      } else {
        generateCylinder(result_cloud_, 0.150, 0.325, cone.position.x, cone.position.y, 100, 100,
                         0);
      }
    }

    if (large_count != 4)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%d expected->%d",
                  large_count, 4);

    // Number of points in the PCD should be height*width
    result_cloud_->width = result_cloud_->points.size();
    result_cloud_->height = 1;

    // Save the generated cloud to a PCD file
    std::string output_pcd_path =
        "../../src/perception/test/perception_integration_tests/results/"
        "accelaration_end_close.pcd";
    if (pcl::io::savePCDFile(output_pcd_path, *result_cloud_) == -1) {
      RCLCPP_ERROR(test_node_->get_logger(), "Failed to save result PCD file.");
    } else {
      RCLCPP_INFO(test_node_->get_logger(), "Result cloud saved to: %s", output_pcd_path.c_str());
    }
  }

  executor.cancel();
}

/**
 * @brief Medium distance to accelaration end test for perception node from rosbag:
 * Accelaration_Testing_Manual-4.mcap
 */
TEST_F(PerceptionIntegrationTest, AccelarationMedium) {
  auto params = load_adapter_parameters();
  rclcpp::Node::SharedPtr perception_node = std::make_shared<Perception>(params);
  ASSERT_NE(perception_node, nullptr) << "Failed to initialize Perception node.";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(test_node_);

  std::string pcd_file_path =
      "../../src/perception/test/perception_integration_tests/point_clouds/"
      "accelaration_end_medium.pcd";
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
    if (cones_result_->cone_array.size() != 12)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%ld expected->%d",
                  cones_result_->cone_array.size(), 12);

    int large_count = 0;
    for (auto cone : cones_result_->cone_array) {
      if (cone.is_large) {
        large_count++;
        generateCylinder(result_cloud_, 0.220, 0.605, cone.position.x, cone.position.y, 100, 100,
                         100);
      } else {
        generateCylinder(result_cloud_, 0.150, 0.325, cone.position.x, cone.position.y, 100, 100,
                         0);
      }
    }

    if (large_count != 4)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%d expected->%d",
                  large_count, 4);

    // Number of points in the PCD should be height*width
    result_cloud_->width = result_cloud_->points.size();
    result_cloud_->height = 1;

    // Save the generated cloud to a PCD file
    std::string output_pcd_path =
        "../../src/perception/test/perception_integration_tests/results/"
        "accelaration_end_medium.pcd";
    if (pcl::io::savePCDFile(output_pcd_path, *result_cloud_) == -1) {
      RCLCPP_ERROR(test_node_->get_logger(), "Failed to save result PCD file.");
    } else {
      RCLCPP_INFO(test_node_->get_logger(), "Result cloud saved to: %s", output_pcd_path.c_str());
    }
  }

  executor.cancel();
}

/**
 * @brief Far distance to accelaration end test for perception node from rosbag:
 * Accelaration_Testing_Manual-4.mcap
 */
TEST_F(PerceptionIntegrationTest, AccelarationFar) {
  auto params = load_adapter_parameters();
  rclcpp::Node::SharedPtr perception_node = std::make_shared<Perception>(params);
  ASSERT_NE(perception_node, nullptr) << "Failed to initialize Perception node.";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(test_node_);

  std::string pcd_file_path =
      "../../src/perception/test/perception_integration_tests/point_clouds/"
      "accelaration_end_far.pcd";
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
    if (cones_result_->cone_array.size() != 14)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%ld expected->%d",
                  cones_result_->cone_array.size(), 14);

    int large_count = 0;
    for (auto cone : cones_result_->cone_array) {
      if (cone.is_large) {
        large_count++;
        generateCylinder(result_cloud_, 0.220, 0.605, cone.position.x, cone.position.y, 100, 100,
                         100);
      } else {
        generateCylinder(result_cloud_, 0.150, 0.325, cone.position.x, cone.position.y, 100, 100,
                         0);
      }
    }

    if (large_count != 4)
      RCLCPP_INFO(test_node_->get_logger(),
                  "Wrong number of large cones: detected->%d expected->%d", large_count, 4);

    // Number of points in the PCD should be height*width
    result_cloud_->width = result_cloud_->points.size();
    result_cloud_->height = 1;

    // Save the generated cloud to a PCD file
    std::string output_pcd_path =
        "../../src/perception/test/perception_integration_tests/results/"
        "accelaration_end_far.pcd";
    if (pcl::io::savePCDFile(output_pcd_path, *result_cloud_) == -1) {
      RCLCPP_ERROR(test_node_->get_logger(), "Failed to save result PCD file.");
    } else {
      RCLCPP_INFO(test_node_->get_logger(), "Result cloud saved to: %s", output_pcd_path.c_str());
    }
  }

  executor.cancel();
}

/**
 * @brief Blind turn test for perception node from rosbag: Closed_Course_Manual-6.mcap
 */
TEST_F(PerceptionIntegrationTest, EnterHairpin) {
  auto params = load_adapter_parameters();
  rclcpp::Node::SharedPtr perception_node = std::make_shared<Perception>(params);
  ASSERT_NE(perception_node, nullptr) << "Failed to initialize Perception node.";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(test_node_);

  std::string pcd_file_path =
      "../../src/perception/test/perception_integration_tests/point_clouds/"
      "enter_hairpin.pcd";
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
    if (cones_result_->cone_array.size() != 10)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%ld expected->%d",
                  cones_result_->cone_array.size(), 10);

    int large_count = 0;
    for (auto cone : cones_result_->cone_array) {
      if (cone.is_large) {
        large_count++;
        generateCylinder(result_cloud_, 0.220, 0.605, cone.position.x, cone.position.y, 100, 100,
                         100);
      } else {
        generateCylinder(result_cloud_, 0.150, 0.325, cone.position.x, cone.position.y, 100, 100,
                         0);
      }
    }

    if (large_count != 0)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%d expected->%d",
                  large_count, 0);

    // Number of points in the PCD should be height*width
    result_cloud_->width = result_cloud_->points.size();
    result_cloud_->height = 1;

    // Save the generated cloud to a PCD file
    std::string output_pcd_path =
        "../../src/perception/test/perception_integration_tests/results/"
        "enter_hairpin.pcd";
    if (pcl::io::savePCDFile(output_pcd_path, *result_cloud_) == -1) {
      RCLCPP_ERROR(test_node_->get_logger(), "Failed to save result PCD file.");
    } else {
      RCLCPP_INFO(test_node_->get_logger(), "Result cloud saved to: %s", output_pcd_path.c_str());
    }
  }

  executor.cancel();
}

/**
 * @brief Turn test for perception node from rosbag: Hard_Course-DV-5.mcap
 */
TEST_F(PerceptionIntegrationTest, TurnStart) {
  auto params = load_adapter_parameters();
  rclcpp::Node::SharedPtr perception_node = std::make_shared<Perception>(params);
  ASSERT_NE(perception_node, nullptr) << "Failed to initialize Perception node.";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(test_node_);

  std::string pcd_file_path =
      "../../src/perception/test/perception_integration_tests/point_clouds/"
      "turn_start.pcd";
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
    if (cones_result_->cone_array.size() != 12)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%ld expected->%d",
                  cones_result_->cone_array.size(), 12);

    int large_count = 0;
    for (auto cone : cones_result_->cone_array) {
      if (cone.is_large) {
        large_count++;
        generateCylinder(result_cloud_, 0.220, 0.605, cone.position.x, cone.position.y, 100, 100,
                         100);
      } else {
        generateCylinder(result_cloud_, 0.150, 0.325, cone.position.x, cone.position.y, 100, 100,
                         0);
      }
    }

    if (large_count != 0)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%d expected->%d",
                  large_count, 0);

    // Number of points in the PCD should be height*width
    result_cloud_->width = result_cloud_->points.size();
    result_cloud_->height = 1;

    // Save the generated cloud to a PCD file
    std::string output_pcd_path =
        "../../src/perception/test/perception_integration_tests/results/"
        "turn_start.pcd";
    if (pcl::io::savePCDFile(output_pcd_path, *result_cloud_) == -1) {
      RCLCPP_ERROR(test_node_->get_logger(), "Failed to save result PCD file.");
    } else {
      RCLCPP_INFO(test_node_->get_logger(), "Result cloud saved to: %s", output_pcd_path.c_str());
    }
  }

  executor.cancel();
}

/**
 * @brief Odd situation test for perception node from rosbag: Hard_Course-DV-5.mcap
 */
TEST_F(PerceptionIntegrationTest, OddStituation) {
  auto params = load_adapter_parameters();
  rclcpp::Node::SharedPtr perception_node = std::make_shared<Perception>(params);
  ASSERT_NE(perception_node, nullptr) << "Failed to initialize Perception node.";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(test_node_);

  std::string pcd_file_path =
      "../../src/perception/test/perception_integration_tests/point_clouds/"
      "odd_situation.pcd";
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
    if (cones_result_->cone_array.size() != 10)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%ld expected->%d",
                  cones_result_->cone_array.size(), 10);

    int large_count = 0;
    for (auto cone : cones_result_->cone_array) {
      if (cone.is_large) {
        large_count++;
        generateCylinder(result_cloud_, 0.220, 0.605, cone.position.x, cone.position.y, 100, 100,
                         100);
      } else {
        generateCylinder(result_cloud_, 0.150, 0.325, cone.position.x, cone.position.y, 100, 100,
                         0);
      }
    }

    if (large_count != 4)
      RCLCPP_INFO(test_node_->get_logger(),
                  "Wrong number of large cones: detected->%d expected->%d", large_count, 4);

    // Number of points in the PCD should be height*width
    result_cloud_->width = result_cloud_->points.size();
    result_cloud_->height = 1;

    // Save the generated cloud to a PCD file
    std::string output_pcd_path =
        "../../src/perception/test/perception_integration_tests/results/"
        "odd_situation.pcd";
    if (pcl::io::savePCDFile(output_pcd_path, *result_cloud_) == -1) {
      RCLCPP_ERROR(test_node_->get_logger(), "Failed to save result PCD file.");
    } else {
      RCLCPP_INFO(test_node_->get_logger(), "Result cloud saved to: %s", output_pcd_path.c_str());
    }
  }

  executor.cancel();
}

/**
 * @brief A fully diagonal path test for perception node from rosbag: Autocross_DV-1.mcap
 */
TEST_F(PerceptionIntegrationTest, DiagonalPath) {
  auto params = load_adapter_parameters();
  rclcpp::Node::SharedPtr perception_node = std::make_shared<Perception>(params);
  ASSERT_NE(perception_node, nullptr) << "Failed to initialize Perception node.";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(test_node_);

  std::string pcd_file_path =
      "../../src/perception/test/perception_integration_tests/point_clouds/"
      "diagonal_path.pcd";
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
    if (cones_result_->cone_array.size() != 9)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%ld expected->c%d",
                  cones_result_->cone_array.size(), 9);

    int large_count = 0;
    for (auto cone : cones_result_->cone_array) {
      if (cone.is_large) {
        large_count++;
        generateCylinder(result_cloud_, 0.220, 0.605, cone.position.x, cone.position.y, 100, 100,
                         100);
      } else {
        generateCylinder(result_cloud_, 0.150, 0.325, cone.position.x, cone.position.y, 100, 100,
                         0);
      }
    }

    if (large_count != 0)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%d expected->%d",
                  large_count, 0);

    // Number of points in the PCD should be height*width
    result_cloud_->width = result_cloud_->points.size();
    result_cloud_->height = 1;

    // Save the generated cloud to a PCD file
    std::string output_pcd_path =
        "../../src/perception/test/perception_integration_tests/results/"
        "diagonal_path.pcd";
    if (pcl::io::savePCDFile(output_pcd_path, *result_cloud_) == -1) {
      RCLCPP_ERROR(test_node_->get_logger(), "Failed to save result PCD file.");
    } else {
      RCLCPP_INFO(test_node_->get_logger(), "Result cloud saved to: %s", output_pcd_path.c_str());
    }
  }

  executor.cancel();
}