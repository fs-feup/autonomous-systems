#include <gtest/gtest.h>

#include "cone_differentiation/least_squares_differentiation.hpp"

/**
 * @brief Fixture for testing the LeastSquaresDifferentiation class.
 */
class LeastSquaresDifferentiationTest : public ::testing::Test {
protected:
  /**
   * @brief Set up the test fixtures.
   */
  void SetUp() override {
    blue_cone = std::make_shared<pcl::PointCloud<PointXYZIR>>();
    yellow_cone = std::make_shared<pcl::PointCloud<PointXYZIR>>();
    pcl_cloud_2_points = std::make_shared<pcl::PointCloud<PointXYZIR>>();
    real_blue_cone = std::make_shared<pcl::PointCloud<PointXYZIR>>();

    blue_cone->points.push_back(PointXYZIR{1.0, 0.0, 0.0, 20, 0});
    blue_cone->points.push_back(PointXYZIR{0.0, 1.0, 1, 20, 0});
    blue_cone->points.push_back(PointXYZIR{0.0, 0.0, 2.0, 21, 0});
    blue_cone->points.push_back(PointXYZIR{0.0060, 0.0060, 4.0, 60.0, 0});
    blue_cone->points.push_back(PointXYZIR{10, 10, 5, 70, 0});
    blue_cone->points.push_back(PointXYZIR{10, 10, 4.5, 65, 0});
    blue_cone->points.push_back(PointXYZIR{10, 10, 7, 49, 0});
    blue_cone->points.push_back(PointXYZIR{10, 10, 9, 30, 0});
    blue_cone->points.push_back(PointXYZIR{10, 10, 10, 20, 0});

    yellow_cone->points.push_back(PointXYZIR{1.0, 0.0, 0.0, 60, 0});
    yellow_cone->points.push_back(PointXYZIR{0.0, 1.0, 0, 70, 0});
    yellow_cone->points.push_back(PointXYZIR{0.0, 0.0, 2.0, 50, 0});
    yellow_cone->points.push_back(PointXYZIR{0.0060, 0.0060, 3.0, 50.0, 0});
    yellow_cone->points.push_back(PointXYZIR{10, 10, 4, 40, 0});
    yellow_cone->points.push_back(PointXYZIR{10, 10, 5, 20, 0});
    yellow_cone->points.push_back(PointXYZIR{10, 10, 6, 15, 0});
    yellow_cone->points.push_back(PointXYZIR{10, 10, 7, 30, 0});
    yellow_cone->points.push_back(PointXYZIR{10, 10, 8, 45, 0});
    yellow_cone->points.push_back(PointXYZIR{10, 10, 9, 60, 0});
    yellow_cone->points.push_back(PointXYZIR{10, 10, 10, 80, 0});

    pcl_cloud_2_points->points.push_back(PointXYZIR{1.0, 0.0, 0.0, 0.5, 0});
    pcl_cloud_2_points->points.push_back(PointXYZIR{0.0, 1.0, 0.0, 1.0, 0});

    real_blue_cone->points.push_back(PointXYZIR{1.0, 0.0, 0.3, 5, 0});
    real_blue_cone->points.push_back(PointXYZIR{1.0, 0.0, 0.3, 1});
    real_blue_cone->points.push_back(PointXYZIR{1.0, 0.0, 0.3, 72});
    real_blue_cone->points.push_back(PointXYZIR{1.0, 0.0, 0.27, 6});
  }

  pcl::PointCloud<PointXYZIR>::Ptr blue_cone;
  pcl::PointCloud<PointXYZIR>::Ptr yellow_cone;
  pcl::PointCloud<PointXYZIR>::Ptr pcl_cloud_2_points;
  pcl::PointCloud<PointXYZIR>::Ptr real_blue_cone;
  const LeastSquaresDifferentiation cone_differentiator;
};

TEST_F(LeastSquaresDifferentiationTest, TestYellowCone) {
  Cluster cluster(yellow_cone);

  cone_differentiator.coneDifferentiation(&cluster);

  ASSERT_EQ(cluster.get_color(), "yellow");
}

TEST_F(LeastSquaresDifferentiationTest, TestBlueCone) {
  Cluster cluster(blue_cone);

  cone_differentiator.coneDifferentiation(&cluster);

  ASSERT_EQ(cluster.get_color(), "blue");
}

TEST_F(LeastSquaresDifferentiationTest, TestUndefinedCone) {
  Cluster cluster(pcl_cloud_2_points);

  cone_differentiator.coneDifferentiation(&cluster);

  ASSERT_EQ(cluster.get_color(), "undefined");
}
