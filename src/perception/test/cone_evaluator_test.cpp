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
  // Create test validators with predefined results
  auto height_validator = std::make_shared<TestConeValidator>(std::vector<double>{0.3, 0.7});
  auto cylinder_validator = std::make_shared<TestConeValidator>(std::vector<double>{0.5, 0.2, 0.1});
  auto npoints_validator = std::make_shared<TestConeValidator>(std::vector<double>{1.0});
  auto displacement_validator =
      std::make_shared<TestConeValidator>(std::vector<double>{0.2, 0.3, 0.4});
  auto deviation_validator = std::make_shared<TestConeValidator>(std::vector<double>{0.5, 0.6});

  // Create a map of validators
  auto cone_validators =
      std::make_shared<std::unordered_map<std::string, std::shared_ptr<ConeValidator>>>();
  cone_validators->emplace("height", height_validator);
  cone_validators->emplace("cylinder", cylinder_validator);
  cone_validators->emplace("npoints", npoints_validator);
  cone_validators->emplace("displacement", displacement_validator);
  cone_validators->emplace("deviation", deviation_validator);

  // Create a map of evaluator weights
  auto evaluator_weights = std::make_shared<Weights>();
  evaluator_weights->height_out = 0.1;
  evaluator_weights->height_in = 0.1;
  evaluator_weights->cylinder_radius = 0.2;
  evaluator_weights->cylinder_height = 0.05;
  evaluator_weights->cylinder_npoints = 0.05;
  evaluator_weights->npoints = 0.25;
  evaluator_weights->displacement_x = 0.05;
  evaluator_weights->displacement_y = 0.05;
  evaluator_weights->displacement_z = 0.05;
  evaluator_weights->deviation_xoy = 0.05;
  evaluator_weights->deviation_z = 0.05;

  double min_confidence = 0.5;
  ConeEvaluator cone_evaluator(cone_validators, evaluator_weights, min_confidence);

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
  // Create test validators with predefined results that yield low confidence
  auto height_validator = std::make_shared<TestConeValidator>(std::vector<double>{0.0, 0.0});
  auto cylinder_validator = std::make_shared<TestConeValidator>(std::vector<double>{0.0, 0.0, 0.0});
  auto npoints_validator = std::make_shared<TestConeValidator>(std::vector<double>{0.0});
  auto displacement_validator =
      std::make_shared<TestConeValidator>(std::vector<double>{0.0, 0.0, 0.0});
  auto deviation_validator = std::make_shared<TestConeValidator>(std::vector<double>{0.0, 0.0});

  // Create a map of validators
  auto cone_validators =
      std::make_shared<std::unordered_map<std::string, std::shared_ptr<ConeValidator>>>();
  cone_validators->emplace("height", height_validator);
  cone_validators->emplace("cylinder", cylinder_validator);
  cone_validators->emplace("npoints", npoints_validator);
  cone_validators->emplace("displacement", displacement_validator);
  cone_validators->emplace("deviation", deviation_validator);

  // Create a map of evaluator weights
  auto evaluator_weights = std::make_shared<Weights>();
  evaluator_weights->height_out = 0.1;
  evaluator_weights->height_in = 0.1;
  evaluator_weights->cylinder_radius = 0.2;
  evaluator_weights->cylinder_height = 0.05;
  evaluator_weights->cylinder_npoints = 0.05;
  evaluator_weights->npoints = 0.25;
  evaluator_weights->displacement_x = 0.05;
  evaluator_weights->displacement_y = 0.05;
  evaluator_weights->displacement_z = 0.05;
  evaluator_weights->deviation_xoy = 0.05;
  evaluator_weights->deviation_z = 0.05;

  double min_confidence = 0.5;
  ConeEvaluator cone_evaluator(cone_validators, evaluator_weights, min_confidence);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  Cluster cluster(cloud);
  Plane ground_plane;

  bool result = cone_evaluator.evaluateCluster(cluster, ground_plane);

  ASSERT_NEAR(cluster.get_confidence(), 0.0, 1e-3);
  ASSERT_EQ(result, false);
}
