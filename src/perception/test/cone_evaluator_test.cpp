#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cone_evaluator/cone_evaluator.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <utils/cluster.hpp>
#include <utils/plane.hpp>

/// Mock ConeValidator class
class TestConeValidator : public ConeValidator {
public:
  TestConeValidator(const std::vector<double>& results) : results_(results) {}

  std::vector<double> coneValidator([[maybe_unused]] Cluster* cluster,
                                    [[maybe_unused]] Plane& ground_plane) const override {
    return results_;
  }

private:
  std::vector<double> results_;
};

/**
 * @brief Test when confidence is sufficient
 */
TEST(ConeEvaluatorTest, EvaluateClusterConfidenceSufficient) {
  auto eval_params = std::make_shared<EvaluatorParameters>();

  // Create test validators with predefined results
  eval_params->height_validator =
      std::make_shared<TestConeValidator>(std::vector<double>{0.3, 0.7});
  eval_params->cylinder_validator =
      std::make_shared<TestConeValidator>(std::vector<double>{0.5, 0.2, 0.1});
  eval_params->npoints_validator = std::make_shared<TestConeValidator>(std::vector<double>{1.0});
  eval_params->displacement_validator =
      std::make_shared<TestConeValidator>(std::vector<double>{0.2, 0.3, 0.4});
  eval_params->deviation_validator =
      std::make_shared<TestConeValidator>(std::vector<double>{0.5, 0.6});

  // Create a map of evaluator weights
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

  eval_params->min_confidence = 0.5;
  ConeEvaluator cone_evaluator(eval_params);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  Cluster cluster(cloud);
  Plane ground_plane;

  bool result = cone_evaluator.evaluateCluster(cluster, ground_plane);

  ASSERT_NEAR(cluster.get_confidence(), 0.565, 1e-4);
  ASSERT_EQ(result, true);
}

/**
 * @brief Test when confidence is insufficient
 */
TEST(ConeEvaluatorTest, EvaluateClusterConfidenceInsufficient) {
  auto eval_params = std::make_shared<EvaluatorParameters>();

  // Create test validators with predefined results
  eval_params->height_validator =
      std::make_shared<TestConeValidator>(std::vector<double>{0.0, 0.0});
  eval_params->cylinder_validator =
      std::make_shared<TestConeValidator>(std::vector<double>{0.0, 0.0, 0.0});
  eval_params->npoints_validator = std::make_shared<TestConeValidator>(std::vector<double>{0.0});
  eval_params->displacement_validator =
      std::make_shared<TestConeValidator>(std::vector<double>{0.0, 0.0, 0.0});
  eval_params->deviation_validator =
      std::make_shared<TestConeValidator>(std::vector<double>{0.0, 0.0});

  // Create a map of evaluator weights
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

  eval_params->min_confidence = 0.5;
  ConeEvaluator cone_evaluator(eval_params);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  Cluster cluster(cloud);
  Plane ground_plane;

  bool result = cone_evaluator.evaluateCluster(cluster, ground_plane);

  ASSERT_NEAR(cluster.get_confidence(), 0.0, 1e-3);
  ASSERT_EQ(result, false);
}
