#include "track_filtering/track_filter.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class TrackFilterTest : public ::testing::Test {
protected:
  pcl::PointCloud<pcl::PointXYZI>::Ptr dummy_cloud;

  void SetUp() override {
    dummy_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::PointXYZI pt;
    pt.x = pt.y = pt.z = pt.intensity = 0;
    dummy_cloud->points.push_back(pt);
  }

  Cluster createCluster(float x, float y, float z = 0.0f) {
    Cluster cluster(dummy_cloud);
    Eigen::Vector4f centroid(x, y, z, 0.0f);
    cluster.set_centroid(centroid);
    return cluster;
  }
};

TEST_F(TrackFilterTest, FiltersConesThatAreTooClose) {
  std::vector<Cluster> clusters = {
      createCluster(0, 0), createCluster(0.1, 0.1),  // First and Second Too Close
      createCluster(5, 5)                            // Distant, valid
  };

  TrackFilter filter(0.5, 10.0, 1);
  filter.filter(clusters);

  ASSERT_EQ(clusters.size(), 1);
  EXPECT_NEAR(clusters[0].get_centroid().x(), 5.0f, 1e-4);
  EXPECT_NEAR(clusters[0].get_centroid().y(), 5.0f, 1e-4);
}

TEST_F(TrackFilterTest, KeepsValidCones) {
  std::vector<Cluster> clusters = {createCluster(0, 0), createCluster(2, 2), createCluster(4, 4)};

  TrackFilter filter(0.5, 5.0, 1);
  filter.filter(clusters);

  ASSERT_EQ(clusters.size(), 3);
}

TEST_F(TrackFilterTest, FiltersConesWithNotEnoughNeighbors) {
  std::vector<Cluster> clusters = {createCluster(0, 0), createCluster(10, 10),
                                   createCluster(20, 20)};

  TrackFilter filter(0.5, 5.0, 2);  // Require at least 2 nearby cones
  filter.filter(clusters);

  ASSERT_EQ(clusters.size(), 0);
}
