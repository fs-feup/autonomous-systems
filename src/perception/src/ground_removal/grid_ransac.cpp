#include "ground_removal/grid_ransac.hpp"

#include <omp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <cmath>
#include <utils/plane.hpp>
#include <vector>

#include "ground_removal/ransac.hpp"

GridRANSAC::GridRANSAC(const double epsilon, const int n_tries)
    : _ransac_(RANSAC(epsilon, n_tries)) {}

double GridRANSAC::get_furthest_point(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  double max_distance = 0.0;
  for (const auto& point : *cloud) {
    double distance = std::sqrt(point.x * point.x + point.y * point.y);
    if (distance > max_distance) {
      max_distance = distance;
    }
  }
  return max_distance;
}

void GridRANSAC::split_point_cloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>& grids,
    const SplitParameters split_params) const {
  grids.clear();

  double max_distance = get_furthest_point(cloud);
  const double angle_increment = split_params.fov_angle / split_params.n_angular_grids;
  const int n_radius_grids = static_cast<int>(max_distance / split_params.radius_resolution) + 1;

  // Matrix Initialization
  grids.resize(n_radius_grids);
  for (int radius = 0; radius < n_radius_grids; ++radius) {
    grids[radius].resize(split_params.n_angular_grids);
    for (int angle = 0; angle < split_params.n_angular_grids; ++angle) {
      grids[radius][angle].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
  }

  // Matrix Population
  for (const auto& point : *cloud) {
    double radius = std::sqrt(point.x * point.x + point.y * point.y);
    double angle = std::atan2(point.y, point.x) * 180.0 / M_PI;

    if (angle < 0) {
      angle += 360.0;
    }

    const int row = static_cast<int>(radius / split_params.radius_resolution);
    const int column = static_cast<int>(angle / angle_increment) % split_params.n_angular_grids;

    if (row < static_cast<int>(grids.size())) {
      grids[row][column]->points.push_back(point);
    }
  }
}

void GridRANSAC::ground_removal(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr ret, Plane& plane,
                                const SplitParameters split_params) const {
  ret->clear();
  plane = Plane(0, 0, 0, 0);

  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> grids;
  split_point_cloud(point_cloud, grids, split_params);

  int count = 0;

#pragma omp parallel for reduction(+ : count) schedule(dynamic)
  for (auto& grid_row : grids) {
    for (auto& grid_cell : grid_row) {
      if (grid_cell->points.size() < 3) continue;

      Plane grid_plane;
      pcl::PointCloud<pcl::PointXYZI>::Ptr grid_ret(new pcl::PointCloud<pcl::PointXYZI>);
      this->_ransac_.ground_removal(grid_cell, grid_ret, grid_plane, split_params);

#pragma omp critical
      {
        *ret += *grid_ret;
        plane += grid_plane;
        count++;
      }
    }
  }

  if (count > 0) {
    plane /= count;
  }
}
