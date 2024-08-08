#include <gtest/gtest.h>

#include <cone_evaluator/distance_predict.hpp>
#include <utils/cluster.hpp>

/**
 * @brief Test fixture for DistanceValidator class.
 *
 */
class DistanceValidatorTest : public ::testing::Test {
 protected:
  /**
   * @brief Set up function to initialize a point cloud.
   */
  void SetUp() override {
    point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

    point_cloud->push_back(pcl::PointXYZI(1.0, 0.0, 0.0, 0.1));
    point_cloud->push_back(pcl::PointXYZI(1.0, 0.0, 0.0, 0.1));
    point_cloud->push_back(pcl::PointXYZI(1.0, 0.0, 0.0, 0.1));
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud;  ///< Point Cloud object for testing.
};

/**
 * @brief Test case to validate a close cone (1 meter distance) Expected low confidence
 */
TEST_F(DistanceValidatorTest, TestDistance1) {
  Cluster cluster = Cluster(point_cloud);
  auto coneEvaluator = new DistancePredict(0.33, 0.2);

  double coneConfidence = coneEvaluator->evaluate_cluster(cluster);

  ASSERT_LE(coneConfidence, 1.0);

  ASSERT_GE(coneConfidence, 0.0);

  ASSERT_LT(coneConfidence, 0.01);

  ASSERT_GT(coneConfidence, 0.001);
}

/**
 * @brief Test case to validate a close cone (2 meter distance) Expected low confidence
 */
TEST_F(DistanceValidatorTest, TestDistance2) {
  Cluster cluster = Cluster(point_cloud);
  auto coneEvaluator = new DistancePredict(0.33, 0.2);

  double coneConfidence = coneEvaluator->evaluate_cluster(cluster);

  ASSERT_LE(coneConfidence, 1.0);

  ASSERT_GE(coneConfidence, 0.0);

  ASSERT_LT(coneConfidence, 0.004);

  ASSERT_GT(coneConfidence, 0.003);
}

/**
 * @brief Test case to validate a close cone (25 meter distance) Expected great confidence
 */
TEST_F(DistanceValidatorTest, TestDistance25) {
  point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

  for (int i = 0; i < 4; i++) {
    point_cloud->push_back(pcl::PointXYZI(25.0, 0.0, 0.0, 0.1));
  }

  Cluster cluster = Cluster(point_cloud);
  auto coneEvaluator = new DistancePredict(0.33, 0.2);

  double coneConfidence = coneEvaluator->evaluate_cluster(cluster);

  ASSERT_LE(coneConfidence, 1.0);

  ASSERT_GE(coneConfidence, 0.0);

  ASSERT_GT(coneConfidence, 0.6);
}
