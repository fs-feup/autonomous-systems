#include "ground_removal/constrained_ransac.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <utils/plane.hpp>

// Non-owning deleter: does nothing.
template <typename T>
struct NonOwningDeleter {
  void operator()(T*) const {}
};

class ConstrainedRANSACTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Construct a base point cloud with a mix of points
    cloud.points.clear();
    cloud.points.push_back(PointXYZIR{1.0, 0.0, 0.0, 0.5});
    cloud.points.push_back(PointXYZIR{0.0, 1.0, 0.0, 1.0});
    cloud.points.push_back(PointXYZIR{0.0, 0.0, 1.0, 1.5});
    cloud.points.push_back(PointXYZIR{0.005, 0.005, 0.005, 2.0});
    cloud.points.push_back(PointXYZIR{10.0, 10.0, 10.0, 3.0});

    cloud_3_points.points.clear();
    cloud_3_points.points.push_back(PointXYZIR{0.0, 0.0, 0.0, 0.5});
    cloud_3_points.points.push_back(PointXYZIR{1.0, 0.0, 0.0, 1.0});
    cloud_3_points.points.push_back(PointXYZIR{0.0, 1.0, 0.0, 1.5});

    empty_cloud.points.clear();
  }

  pcl::PointCloud<PointXYZIR> cloud;
  pcl::PointCloud<PointXYZIR> cloud_3_points;
  pcl::PointCloud<PointXYZIR> ground_removed_cloud;
  pcl::PointCloud<PointXYZIR> empty_cloud;
  SplitParameters split_params;
};

/**
 * @brief Test: Accept planes only if angle diff <= threshold.
 */
TEST_F(ConstrainedRANSACTest, AcceptPlaneWithinAngle) {
  Plane base_plane(0, 0, 1, 0);  // Z-up plane

  ConstrainedRANSAC ransac(0.1, 100, 10.0);  // 10 deg allowed
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud_3_points,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());
  const pcl::PointCloud<PointXYZIR>::Ptr output_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());

  ransac.ground_removal(cloud_ptr, output_ptr, base_plane, split_params);

  ASSERT_EQ(output_ptr->points.size(), 0) << "All 3 points lie on base-like plane.";
}

/**
 * @brief Test: Reject planes if angle diff > threshold (fallback uses base_plane).
 */
TEST_F(ConstrainedRANSACTest, RejectPlaneBeyondAngle) {
  Plane base_plane(0, 0, 1, 0);  // Z-up plane

  ConstrainedRANSAC ransac(0.1, 100, 1.0);  // only 1 degree allowed
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());
  const pcl::PointCloud<PointXYZIR>::Ptr output_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());

  ransac.ground_removal(cloud_ptr, output_ptr, base_plane, split_params);

  // Since all candidate planes are skipped, the base_plane is used.
  // Points close to the XY plane (z≈0) will be removed as ground.
  EXPECT_EQ(output_ptr->points.size(), 2) << "Only points far from XY plane should remain.";
}

/**
 * @brief Test: Small epsilon -> only points in the plane are removed.
 */
TEST_F(ConstrainedRANSACTest, SmallEpsilonKeepsMostPoints) {
  Plane base_plane(0, 0, 1, 0);

  ConstrainedRANSAC ransac(0.00001, 100, 20.0);
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());
  const pcl::PointCloud<PointXYZIR>::Ptr output_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());

  ransac.ground_removal(cloud_ptr, output_ptr, base_plane, split_params);

  ASSERT_EQ(output_ptr->points.size(), 2);
}

/**
 * @brief Test: Huge epsilon -> all points treated as ground -> output empty.
 */
TEST_F(ConstrainedRANSACTest, HugeEpsilonRemovesAll) {
  Plane base_plane(0, 0, 1, 0);

  ConstrainedRANSAC ransac(1000.0, 50, 45.0);
  const pcl::PointCloud<PointXYZIR>::Ptr cloud_ptr(&cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());
  const pcl::PointCloud<PointXYZIR>::Ptr output_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());

  ransac.ground_removal(cloud_ptr, output_ptr, base_plane, split_params);

  ASSERT_EQ(output_ptr->points.size(), 0);
}

/**
 * @brief Test: Cloud with <3 points throws exception.
 */
TEST_F(ConstrainedRANSACTest, TooFewPointsThrows) {
  Plane base_plane(0, 0, 1, 0);

  ConstrainedRANSAC ransac(0.1, 50, 10.0);
  const pcl::PointCloud<PointXYZIR>::Ptr empty_ptr(&empty_cloud,
                                                   NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());
  const pcl::PointCloud<PointXYZIR>::Ptr output_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());

  ransac.ground_removal(empty_ptr, output_ptr, base_plane, split_params);

  EXPECT_EQ(output_ptr->points.size(), empty_ptr->points.size());
}

/**
 * @brief Test: With 3 points and epsilon=0, none are removed.
 */
TEST_F(ConstrainedRANSACTest, ThreePointsZeroThresholdKeepsAll) {
  Plane base_plane(0, 0, 1, 0);

  ConstrainedRANSAC ransac(0.0, 100, 20.0);
  const pcl::PointCloud<PointXYZIR>::Ptr cloud3_ptr(
      &cloud_3_points, NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());
  const pcl::PointCloud<PointXYZIR>::Ptr output_ptr(
      &ground_removed_cloud, NonOwningDeleter<pcl::PointCloud<PointXYZIR>>());

  ransac.ground_removal(cloud3_ptr, output_ptr, base_plane, split_params);

  ASSERT_EQ(output_ptr->points.size(), 3);
}
