#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cone_evaluator/cone_evaluator.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <utils/cluster.hpp>
#include <utils/plane.hpp>

// Non-owning deleter: does nothing.
template <typename T>
struct NonOwningDeleter {
  void operator()(T*) const {}
};

/// Mock ConeValidator class
class TestConeValidator : public ConeValidator {
public:
  TestConeValidator(const EvaluatorResults& results) : results_(results) {}

  void coneValidator([[maybe_unused]] Cluster* cluster, [[maybe_unused]] EvaluatorResults* results,
                     [[maybe_unused]] Plane& ground_plane) const override {
    *results = results_;
  }

private:
  EvaluatorResults results_;
};

std::shared_ptr<EvaluatorParameters> createTestEvaluatorParams(const EvaluatorResults& fake_results,
                                                               float min_confidence = 0.5f) {
  auto eval_params = std::make_shared<EvaluatorParameters>();

  eval_params->height_validator = std::make_shared<TestConeValidator>(fake_results);
  eval_params->cylinder_validator = std::make_shared<TestConeValidator>(fake_results);
  eval_params->npoints_validator = std::make_shared<TestConeValidator>(fake_results);
  eval_params->displacement_validator = std::make_shared<TestConeValidator>(fake_results);
  eval_params->deviation_validator = std::make_shared<TestConeValidator>(fake_results);

  // Set evaluation weights
  eval_params->height_out_weight = 0.1;
  eval_params->height_in_weight = 0.1;
  eval_params->cylinder_radius_weight = 0.2;
  eval_params->cylinder_height_weight = 0.05;
  eval_params->cylinder_npoints_weight = 0.05;
  eval_params->npoints_weight = 0.25;
  eval_params->displacement_x_weight = 0.05;
  eval_params->displacement_y_weight = 0.05;
  eval_params->displacement_z_weight = 0.05;
  eval_params->deviation_xoy_weight = 0.05;
  eval_params->deviation_z_weight = 0.05;

  eval_params->min_confidence = min_confidence;
  return eval_params;
}

/**
 * @brief Test when confidence is sufficient
 */
TEST(ConeEvaluatorTest, EvaluateClusterConfidenceSufficient) {
  EvaluatorResults fake_results;
  fake_results.cylinder_out_distance_xy_small = 0.7;
  fake_results.cylinder_out_distance_xy_large = 0.7;
  fake_results.cylinder_n_large_points = 0;
  fake_results.cylinder_n_out_points = 0;
  fake_results.cylinder_out_distance_z_large = 0.9;
  fake_results.cylinder_out_distance_z_small = 0.9;

  fake_results.deviation_xoy = 0.9;
  fake_results.deviation_z = 0.9;

  fake_results.height_out_ratio_small = 1.0;
  fake_results.height_out_ratio_large = 1.0;
  fake_results.height_in_ratio_large = 0.8;
  fake_results.height_in_ratio_small = 0.8;
  fake_results.height_large = false;

  fake_results.n_points = 1.0;

  fake_results.displacement_x = 0.8;
  fake_results.displacement_y = 0.8;
  fake_results.displacement_z = 0.8;

  auto eval_params = createTestEvaluatorParams(fake_results);

  ConeEvaluator cone_evaluator(eval_params);

  // Create a stack-allocated point cloud.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  // Wrap the stack object with a non-owning shared pointer.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  Cluster cluster(cloud_ptr);
  Plane ground_plane;

  const bool result = cone_evaluator.evaluateCluster(cluster, ground_plane);

  ASSERT_NEAR(cluster.get_confidence(), 0.565, 1e-4);
  ASSERT_EQ(result, true);
}

/**
 * @brief Test when confidence is insufficient
 */
TEST(ConeEvaluatorTest, EvaluateClusterConfidenceInsufficient) {
  EvaluatorResults fake_results = {};

  auto eval_params = createTestEvaluatorParams(fake_results);

  ConeEvaluator cone_evaluator(eval_params);

  // Create a stack-allocated point cloud.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  // Wrap the stack object with a non-owning shared pointer.
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      &cloud, NonOwningDeleter<pcl::PointCloud<pcl::PointXYZI>>());

  Cluster cluster(cloud_ptr);
  Plane ground_plane;

  const bool result = cone_evaluator.evaluateCluster(cluster, ground_plane);

  ASSERT_NEAR(cluster.get_confidence(), 0.0, 1e-3);
  ASSERT_EQ(result, false);
}
