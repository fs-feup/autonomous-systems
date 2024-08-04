#include "clustering/dbscan.hpp"
#include "cone_validator/z_score_validator.hpp"

#include <gtest/gtest.h>

class ZScoreValidatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    clusters = {};
  }

  void add_cluster(float centroid_x, float centroid_y){
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    point_cloud->push_back(pcl::PointXYZI{centroid_x, centroid_y, 42, 42});
    clusters.push_back(Cluster(point_cloud));
  }

  std::vector<Cluster> clusters;
  Plane plane;
};

TEST_F(ZScoreValidatorTest, SinglePointPerfectZScores) {
  auto validator = new ZScoreValidator(1, 1, 1, 1);
  add_cluster(-1, 0);
  Cluster::set_z_scores(clusters);

  ASSERT_TRUE(validator->coneValidator(&clusters[0], plane));
  ASSERT_EQ(clusters[0].get_z_score_x(), 1);
  ASSERT_EQ(clusters[0].get_z_score_y(), 1);
}

TEST_F(ZScoreValidatorTest, TwoPointsPerfectZScores) {
  auto validator = new ZScoreValidator(1, 1, 1, 1);
  add_cluster(-1, 0);
  add_cluster(1, 0);
  Cluster::set_z_scores(clusters);

  ASSERT_TRUE(validator->coneValidator(&clusters[0], plane));
  ASSERT_EQ(clusters[0].get_z_score_x(), 1);
  ASSERT_EQ(clusters[0].get_z_score_y(), 1);
  ASSERT_EQ(clusters[1].get_z_score_x(), 1);
  ASSERT_EQ(clusters[1].get_z_score_y(), 1);
}



TEST_F(ZScoreValidatorTest, PerfectZScore) {
  auto validator = new ZScoreValidator(1, 1, 1, 1);
  add_cluster(-1, 0);
  add_cluster(1, 0);
  add_cluster(-1, 1);
  add_cluster(1, 1);
  Cluster::set_z_scores(clusters);

  for (int i = 0; i < 4; i++) {
    ASSERT_TRUE(validator->coneValidator(&clusters[i], plane));
  }
}

TEST_F(ZScoreValidatorTest, PerfectZScoreWith1Outlier) {
  auto validator = new ZScoreValidator(1, 1, 1, 1);
  add_cluster(-1, 0);
  add_cluster(1, 0);
  add_cluster(-1, 1);
  add_cluster(1, 1);
  add_cluster(5, 5);
  Cluster::set_z_scores(clusters);

  for (int i = 0; i < 5; i++) {
    ASSERT_FALSE(validator->coneValidator(&clusters[i], plane));
  }
}

TEST_F(ZScoreValidatorTest, PerfectZScoreWith1Outlier2) {
  auto validator = new ZScoreValidator(0, 1, 0, 1);

  for (int i = 0; i < 20; i++){
    add_cluster(-1, 0);
    add_cluster(1, 0);
    add_cluster(-1, 1);
    add_cluster(1, 1);
  }
  add_cluster(8, 8);
  Cluster::set_z_scores(clusters);
  
  ASSERT_TRUE(validator->coneValidator(&clusters[0], plane));
  ASSERT_TRUE(validator->coneValidator(&clusters[1], plane));
  ASSERT_TRUE(validator->coneValidator(&clusters[2], plane));
  ASSERT_TRUE(validator->coneValidator(&clusters[3], plane));
  ASSERT_FALSE(validator->coneValidator(&clusters[80], plane));
}