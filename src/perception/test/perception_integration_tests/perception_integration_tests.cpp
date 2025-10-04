#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <custom_interfaces/msg/cone_array.hpp>
#include <custom_interfaces/msg/perception_output.hpp>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "perception/perception_node.hpp"

/**
 *@brief Helper function to load ground truth cone positions from a file.
 *
 */
bool loadGroundTruth(const std::string& file_path,
                     std::vector<std::tuple<float, float, bool, bool>>& ground_truth) {
  std::ifstream file(file_path);
  if (!file.is_open()) return false;

  float x, y;
  int is_large;
  while (file >> x >> y >> is_large) {
    ground_truth.emplace_back(x, y, is_large, false);
  }
  return true;
}

/**
 *@brief Helper function to compute correctness for a specific test case.
 *
 */
double computeCorrectness(const custom_interfaces::msg::ConeArray::SharedPtr& detected,
                          std::vector<std::tuple<float, float, bool, bool>> ground_truth,
                          float tolerance = 0.4) {
  int true_positives = 0;
  int false_positives = 0;

  for (const auto& cone : detected->cone_array) {
    bool found = false;
    for (auto& [gt_x, gt_y, gt_is_large, gt_found] : ground_truth) {
      float distance = std::hypot(cone.position.x - gt_x, cone.position.y - gt_y);
      if (distance < tolerance && cone.is_large == gt_is_large && !gt_found) {
        true_positives++;
        gt_found = true;
        found = true;
        break;
      }
    }
    if (!found) false_positives++;
  }

  // Calculate false negatives
  int false_negatives = std::count_if(ground_truth.begin(), ground_truth.end(),
                                      [](const auto& gt) { return !std::get<3>(gt); });

  // Compute precision and recall
  double precision =
      (true_positives == 0) ? 0.0 : (double)true_positives / (true_positives + false_positives);
  double recall =
      (true_positives == 0) ? 0.0 : (double)true_positives / (true_positives + false_negatives);

  // Avoid division by zero
  if (precision == 0.0 && recall == 0.0) return 0.0;

  // Compute F1-score
  double f1_score = 2.0 * (precision * recall) / (precision + recall);
  return f1_score * 100.0;  // Return as percentage
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
  rclcpp::Subscription<custom_interfaces::msg::PerceptionOutput>::SharedPtr cones_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_removed_cloud_subscriber_;
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

    // Subscriber for ground removed cloud (msg is shown in the respective results file)
    pcl_removed_cloud_subscriber_ = test_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/ground_removed_cloud", 1,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          pcl::fromROSMsg(*msg, *result_cloud_);
        });

    // Subscriber for output cones
    cones_subscriber_ = test_node_->create_subscription<custom_interfaces::msg::PerceptionOutput>(
        "/perception/cones", 1,
        [this](const custom_interfaces::msg::PerceptionOutput::SharedPtr msg) {
          cones_result_ = std::make_shared<custom_interfaces::msg::ConeArray>(msg->cones);
          cones_received_ = true;
        });

    cones_received_ = false;
    result_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  void TearDown() override { rclcpp::shutdown(); }

  void publish_pcd(const std::string& input_pcd_path) {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_pcd_path, pcl_cloud) == -1) {
      throw std::runtime_error("Could not load PCD file: " + input_pcd_path);
    }

    // Convert PCL cloud to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pcl_cloud, msg);
    msg.header.frame_id = "hesai_lidar";
    msg.header.stamp = test_node_->now();

    // Publish the message
    pcl_publisher_->publish(msg);
  }

  bool waitForCones(rclcpp::executors::SingleThreadedExecutor& executor, int timeout_sec = 2) {
    auto start_time = std::chrono::steady_clock::now();
    while (!cones_received_ &&
           std::chrono::steady_clock::now() - start_time < std::chrono::seconds(timeout_sec)) {
      executor.spin_some();
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    return cones_received_;
  }

  void writeResults(std::string output_pcd_path, int cone_gt, int large_cone_gt) {
    if ((int)cones_result_->cone_array.size() != cone_gt)
      RCLCPP_INFO(test_node_->get_logger(), "Wrong number of cones: detected->%ld expected->%d",
                  cones_result_->cone_array.size(), cone_gt);

    int large_count = 0;
    for (auto cone : cones_result_->cone_array) {
      if (cone.is_large) {
        large_count++;
        generateCylinder(result_cloud_, 0.220, 0.605, cone.position.x, cone.position.y, 50, 50, 14);
      } else {
        generateCylinder(result_cloud_, 0.150, 0.325, cone.position.x, cone.position.y, 50, 50, 36);
      }
    }

    if (large_count != large_cone_gt)
      RCLCPP_INFO(test_node_->get_logger(),
                  "Wrong number of large cones: detected->%d expected->%d", large_count,
                  large_cone_gt);
    // Number of points in the PCD should be height*width
    result_cloud_->width = result_cloud_->points.size();
    result_cloud_->height = 1;

    // Save the generated cloud to a PCD file
    if (pcl::io::savePCDFile(output_pcd_path, *result_cloud_) == -1) {
      RCLCPP_ERROR(test_node_->get_logger(), "Failed to save result PCD file.");
    } else {
      RCLCPP_INFO(test_node_->get_logger(), "Result cloud saved to: %s", output_pcd_path.c_str());
    }
  }

  void runTest(const std::string test_name, const int cone_gt, const int large_cone_gt,
               const int min_correctness, const uint8_t mission) {
    auto params = Perception::load_config();
    params.default_mission_ = mission;

    for (const auto& [mission, trim] : *params.fov_trim_map_) {
      trim->set_lidar_rotation(90);
      trim->set_lidar_pitch(-3.9);
    }

    rclcpp::Node::SharedPtr perception_node = std::make_shared<Perception>(params);
    ASSERT_NE(perception_node, nullptr) << "Failed to initialize Perception node.";

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(perception_node);
    executor.add_node(test_node_);

    std::string input_pcd_path =
        "../../src/perception/test/perception_integration_tests/point_clouds/" + test_name + ".pcd";
    ASSERT_TRUE(std::filesystem::exists(input_pcd_path))
        << "PCD file does not exist: " << input_pcd_path;
    std::string output_pcd_path =
        "../../src/perception/test/perception_integration_tests/results/" + test_name + ".pcd";
    std::string gt_txt_path =
        "../../src/perception/test/perception_integration_tests/ground_truths/" + test_name +
        ".txt";
    ASSERT_TRUE(std::filesystem::exists(gt_txt_path))
        << "Ground truth file does not exist: " << gt_txt_path;

    try {
      publish_pcd(input_pcd_path);
    } catch (const std::exception& e) {
      FAIL() << "Failed to publish PCD: " << e.what();
    }

    ASSERT_TRUE(waitForCones(executor));

    writeResults(output_pcd_path, cone_gt, large_cone_gt);

    std::vector<std::tuple<float, float, bool, bool>> ground_truth;
    ASSERT_TRUE(loadGroundTruth(gt_txt_path, ground_truth)) << "Failed to load ground truth file.";

    // --- DEBUG PRINT START ---
    std::cout << "\n==========================" << std::endl;
    std::cout << " Test: " << test_name << std::endl;
    std::cout << " Expected cones: " << cone_gt << " (large: " << large_cone_gt << ")" << std::endl;

    std::cout << "--- Ground Truth Cones ---" << std::endl;
    for (const auto& [x, y, is_large, _] : ground_truth) {
      std::cout << "GT -> x=" << x << ", y=" << y << ", large=" << (is_large ? "true" : "false")
                << std::endl;
    }

    std::cout << "--- Detected Cones ---" << std::endl;
    if (!cones_result_) {
      std::cout << "No cones detected!" << std::endl;
    } else {
      for (const auto& cone : cones_result_->cone_array) {
        std::cout << "Detected -> x=" << cone.position.x << ", y=" << cone.position.y
                  << ", large=" << (cone.is_large ? "true" : "false") << std::endl;
      }
    }
    std::cout << "==========================\n" << std::endl;
    // --- DEBUG PRINT END ---

    double correctness = computeCorrectness(cones_result_, ground_truth);
    RCLCPP_INFO(test_node_->get_logger(), "Correctness score: %.2f%%", correctness);
    EXPECT_GT(correctness, min_correctness)
        << "Cone detection correctness below acceptable threshold.";

    executor.cancel();
  }
};

/**
 * @brief Straight line test for perception node from rosbag: Accelaration_Testing_DV_1B.mcap
 */
TEST_F(PerceptionIntegrationTest, StraightLine) { runTest("straight_line", 20, 4, 80, 1); }

/**
 * @brief Close to accelaration end test for perception node from rosbag:
 * Accelaration_Testing_Manual-4.mcap
 */
TEST_F(PerceptionIntegrationTest, AccelerationEndClose) {
  runTest("acceleration_end_close", 12, 4, 80, 1);
}

/**
 * @brief Medium distance to accelaration end test for perception node from rosbag:
 * Accelaration_Testing_Manual-4.mcap
 */
TEST_F(PerceptionIntegrationTest, AccelerationEndMedium) {
  runTest("acceleration_end_medium", 14, 4, 80, 1);
}

/**
 * @brief Far distance to accelaration end test for perception node from rosbag:
 * Accelaration_Testing_Manual-4.mcap
 */
TEST_F(PerceptionIntegrationTest, AccelerationFar) {
  runTest("acceleration_end_far", 16, 4, 80, 1);
}

/**
 * @brief Blind turn test for perception node from rosbag: Closed_Course_Manual-6.mcap
 */
TEST_F(PerceptionIntegrationTest, EnterHairpin) { runTest("enter_hairpin", 16, 0, 70, 4); }

/**
 * @brief Turn test for perception node from rosbag: Hard_Course-DV-5.mcap
 */
TEST_F(PerceptionIntegrationTest, TurnStart) { runTest("turn_start", 33, 4, 70, 4); }

/**
 * @brief Odd situation test for perception node from rosbag: Hard_Course-DV-5.mcap
 */
TEST_F(PerceptionIntegrationTest, OddStituation) { runTest("odd_situation", 33, 4, 70, 4); }

/**
 * @brief A fully diagonal path test for perception node from rosbag: Autocross_DV-1.mcap
 */
TEST_F(PerceptionIntegrationTest, DiagonalPath) { runTest("diagonal_path", 26, 0, 80, 4); }