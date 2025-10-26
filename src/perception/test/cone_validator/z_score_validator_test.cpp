#include "cone_validator/z_score_validator.hpp"

#include <gtest/gtest.h>

#include <memory>

#include "clustering/dbscan.hpp"

/**
 * @brief Fixture for testing the ZScoreValidatorTest class.
 */
class ZScoreValidatorTest : public ::testing::Test {
public:
  /**
   * @brief Set up the test fixtures.
   */
  void SetUp() override { _clusters_ = {}; }

  void add_cluster(float centroid_x, float centroid_y) {
    auto point_cloud = std::make_shared<pcl::PointCloud<PointXYZIR>>();
    point_cloud->push_back(PointXYZIR{centroid_x, centroid_y, 42, 42});
    _clusters_.emplace_back(point_cloud);
  }

  std::vector<Cluster> _clusters_;
  Plane _plane_;
};

/**
 * @brief Test case to validate the a vector with only one cluster
 */
TEST_F(ZScoreValidatorTest, SinglePointPerfectZScores) {
  auto validator = ZScoreValidator(1, 1, 1, 1);
  add_cluster(-1, 0);
  Cluster::set_z_scores(_clusters_);

  ASSERT_TRUE(validator.coneValidator(&_clusters_[0], _plane_));
  ASSERT_EQ(_clusters_[0].get_z_score_x(), 1);
  ASSERT_EQ(_clusters_[0].get_z_score_y(), 1);
}

/**
 * @brief Test case to validate the a vector with only two clusters
 */
TEST_F(ZScoreValidatorTest, TwoPointsPerfectZScores) {
  auto validator = ZScoreValidator(1, 1, 1, 1);
  add_cluster(-1, 0);
  add_cluster(1, 0);
  Cluster::set_z_scores(_clusters_);

  ASSERT_TRUE(validator.coneValidator(&_clusters_[0], _plane_));
  ASSERT_EQ(_clusters_[0].get_z_score_x(), 1);
  ASSERT_EQ(_clusters_[0].get_z_score_y(), 1);
  ASSERT_EQ(_clusters_[1].get_z_score_x(), 1);
  ASSERT_EQ(_clusters_[1].get_z_score_y(), 1);
}

/**
 * @brief Test case to validate a perfectly distributed vector of clusters
 *
 */
TEST_F(ZScoreValidatorTest, PerfectZScore) {
  auto validator = ZScoreValidator(1, 1, 1, 1);
  add_cluster(-1, 0);
  add_cluster(1, 0);
  add_cluster(-1, 1);
  add_cluster(1, 1);
  Cluster::set_z_scores(_clusters_);

  for (int i = 0; i < 4; i++) {
    ASSERT_TRUE(validator.coneValidator(&_clusters_[i], _plane_));
  }
}

/**
 * @brief Test case to validate a perfectly distributed vector of clusters with 1 outlier (not
 * perfect)
 *
 */
TEST_F(ZScoreValidatorTest, PerfectZScoreWith1Outlier) {
  auto validator = ZScoreValidator(1, 1, 1, 1);
  add_cluster(-1, 0);
  add_cluster(1, 0);
  add_cluster(-1, 1);
  add_cluster(1, 1);
  add_cluster(5, 5);
  Cluster::set_z_scores(_clusters_);

  for (int i = 0; i < 5; i++) {
    ASSERT_FALSE(validator.coneValidator(&_clusters_[i], _plane_));
  }
}

/**
 * @brief Test case to validate a perfectly distributed vector of clusters with 1 outlier (all
 * points except the outlier)
 *
 */
TEST_F(ZScoreValidatorTest, PerfectZScoreWith1Outlier2) {
  auto validator = ZScoreValidator(0, 1, 0, 1);

  for (int i = 0; i < 20; i++) {
    add_cluster(-1, 0);
    add_cluster(1, 0);
    add_cluster(-1, 1);
    add_cluster(1, 1);
  }
  add_cluster(8, 8);
  Cluster::set_z_scores(_clusters_);

  ASSERT_TRUE(validator.coneValidator(&_clusters_[0], _plane_));
  ASSERT_TRUE(validator.coneValidator(&_clusters_[1], _plane_));
  ASSERT_TRUE(validator.coneValidator(&_clusters_[2], _plane_));
  ASSERT_TRUE(validator.coneValidator(&_clusters_[3], _plane_));
  ASSERT_FALSE(validator.coneValidator(&_clusters_[80], _plane_));
}
