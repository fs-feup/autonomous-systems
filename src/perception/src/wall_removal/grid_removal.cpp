#include "wall_removal/grid_removal.hpp"

GridWallRemoval::GridWallRemoval(double angle, double radius, double start_augmentation,
                                 double radius_augmentation, double fov, int max_points_per_cluster)
    : grid_geometry_(angle, radius, start_augmentation, radius_augmentation, fov),
      max_points_per_cluster_(max_points_per_cluster) {}

void GridWallRemoval::remove_walls(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
                                   sensor_msgs::msg::PointCloud2::SharedPtr& output_cloud) const {
  output_cloud->header = point_cloud->header;
  output_cloud->height = 1;
  output_cloud->is_bigendian = point_cloud->is_bigendian;
  output_cloud->is_dense = point_cloud->is_dense;
  output_cloud->point_step = point_cloud->point_step;
  output_cloud->row_step = 0;
  output_cloud->fields = point_cloud->fields;
  output_cloud->width = 0;
  output_cloud->data.resize(point_cloud->width * point_cloud->point_step);

  const auto& cloud_data = point_cloud->data;
  const size_t num_points = point_cloud->width * point_cloud->height;
  if (num_points == 0) return;

  std::unordered_map<GridIndex, std::vector<size_t>> grid_map;

  for (size_t i = 0; i < num_points; ++i) {
    float x = *reinterpret_cast<const float*>(&cloud_data[PointX(i)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[PointY(i)]);

    int slice = grid_geometry_.get_slice_index(x, y);
    int bin_idx = grid_geometry_.get_bin_index(x, y);
    if (slice == -1) continue;  // Out of FOV
    grid_map[{slice, bin_idx}].push_back(i);
  }

  std::unordered_map<GridIndex, bool> visited;
  for (auto& [cell, points] : grid_map) {
    if (visited[cell]) continue;
    visited[cell] = true;

    std::queue<GridIndex> q;
    q.push(cell);
    std::vector<size_t> cluster_points;

    while (!q.empty()) {
      GridIndex current = q.front();
      q.pop();

      auto it = grid_map.find(current);
      if (it == grid_map.end()) continue;

      for (size_t idx : it->second) cluster_points.push_back(idx);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          GridIndex neighbor{current.x + dx, current.y + dy};
          if (visited[neighbor]) continue;
          if (grid_map.find(neighbor) != grid_map.end()) {
            visited[neighbor] = true;
            q.push(neighbor);
          }
        }
      }
    }

    if (static_cast<int>(cluster_points.size()) < max_points_per_cluster_) {
      for (size_t idx : cluster_points) {
        size_t write_idx = output_cloud->width;
        std::memcpy(&output_cloud->data[write_idx * POINT_STEP], &cloud_data[idx * POINT_STEP],
                    POINT_STEP);
        output_cloud->width++;
      }
    }
  }

  output_cloud->data.resize(output_cloud->width * POINT_STEP);
  output_cloud->row_step = output_cloud->width * POINT_STEP;
}