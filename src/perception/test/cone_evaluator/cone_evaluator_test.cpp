#include <gtest/gtest.h>

#include <cone_evaluator/cone_evaluator.hpp>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <test_utils/pointcloud2_helper.hpp>
#include <utils/cluster.hpp>
#include <utils/evaluator_parameters.hpp>
#include <utils/ground_grid.hpp>

EvaluatorParameters make_default_evaluator_params() {
  EvaluatorParameters params;
  params.small_cone_width = 0.25;
  params.large_cone_width = 0.35;
  params.small_cone_height = 0.35;
  params.large_cone_height = 0.45;
  params.n_out_points_ratio = 0.2;
  params.max_distance_from_ground_min = 0.10;
  params.max_distance_from_ground_max = 0.20;
  params.n_points_intial_max = 20;
  params.n_points_intial_min = 5;
  params.n_points_max_distance_reduction = 0.5;
  params.n_points_min_distance_reduction = 0.2;
  return params;
}

/**
 * @brief Should return true when cluster is close to ground.
 */
TEST(ConeEvaluatorTest, CloseToGroundTrue) {
  auto params = std::make_shared<EvaluatorParameters>(make_default_evaluator_params());
  ConeEvaluator evaluator(params);
  GroundGrid ground_grid(10.0, 1.0, 1.0, 0.0, 0.0, 360.0);

  std::vector<std::array<float, 5>> pts = {{1.0, 2.0, 0.0, 0.1, 0}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  std::vector<int> indices = {0};
  Cluster cluster(input_cloud, indices);
  ground_grid.set_ground_height(1.0, 2.0, 0.0f);

  ASSERT_TRUE(evaluator.close_to_ground(cluster, ground_grid));
}

/**
 * @brief Should return false when cluster is far from ground.
 */
TEST(ConeEvaluatorTest, CloseToGroundFalse) {
  auto params = std::make_shared<EvaluatorParameters>(make_default_evaluator_params());
  ConeEvaluator evaluator(params);
  GroundGrid ground_grid(10.0, 1.0, 1.0, 0.0, 0.0, 360.0);

  std::vector<std::array<float, 5>> pts = {{1.0, 2.0, 1.0, 0.1, 0}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  std::vector<int> indices = {0};
  Cluster cluster(input_cloud, indices);
  ground_grid.set_ground_height(1.0, 2.0, 0.0f);

  ASSERT_FALSE(evaluator.close_to_ground(cluster, ground_grid));
}

/**
 * @brief Should return true when cluster fits cone cylinder dimensions.
 */
TEST(ConeEvaluatorTest, CylinderFitsConeTrue) {
  auto params = std::make_shared<EvaluatorParameters>(make_default_evaluator_params());
  ConeEvaluator evaluator(params);

  std::vector<std::array<float, 5>> pts = {{0.10, 0.05, 0.0, 0.1, 0}, {0.15, 0.0, 0.2, 0.1, 0}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  std::vector<int> indices = {0, 1};
  Cluster cluster(input_cloud, indices);
  ASSERT_TRUE(evaluator.cylinder_fits_cone(cluster));
}

/**
 * @brief Should return false when cluster does not fit cone cylinder dimensions.
 */
TEST(ConeEvaluatorTest, CylinderFitsConeFalse) {
  auto params = std::make_shared<EvaluatorParameters>(make_default_evaluator_params());
  ConeEvaluator evaluator(params);

  std::vector<std::array<float, 5>> pts = {{0.0, 0.0, 0.0, 0.1, 0}, {1.0, 0.0, 2.0, 0.1, 0}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  std::vector<int> indices = {0, 1};
  Cluster cluster(input_cloud, indices);
  ASSERT_FALSE(evaluator.cylinder_fits_cone(cluster));
}

/**
 * @brief Should return true when cluster has a valid number of points.
 */
TEST(ConeEvaluatorTest, NPointsValidTrue) {
  auto params = std::make_shared<EvaluatorParameters>(make_default_evaluator_params());
  ConeEvaluator evaluator(params);

  std::vector<std::array<float, 5>> pts = {{0.0, 0.0, 0.0, 0.1, 0}, {0.1, 0.0, 0.0, 0.1, 0},
                                           {0.2, 0.0, 0.0, 0.1, 0}, {0.3, 0.0, 0.0, 0.1, 0},
                                           {0.4, 0.0, 0.0, 0.1, 0}, {0.5, 0.0, 0.0, 0.1, 0}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  std::vector<int> indices = {0, 1, 2, 3, 4, 5};
  Cluster cluster(input_cloud, indices);
  ASSERT_TRUE(evaluator.npoints_valid(cluster));
}

/**
 * @brief Should return false when cluster does not have a valid number of points.
 */
TEST(ConeEvaluatorTest, NPointsValidFalse) {
  auto params = std::make_shared<EvaluatorParameters>(make_default_evaluator_params());
  ConeEvaluator evaluator(params);

  std::vector<std::array<float, 5>> pts = {{0.0, 0.0, 0.0, 0.1, 0}, {0.1, 0.0, 0.0, 0.1, 0}};
  auto input_cloud = test_utils::make_lidar_pointcloud2(pts);
  std::vector<int> indices = {0, 1};
  Cluster cluster(input_cloud, indices);
  ASSERT_FALSE(evaluator.npoints_valid(cluster));
}
