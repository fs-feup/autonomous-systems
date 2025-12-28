#include "wall_removal/grid_removal.hpp"

GridWallRemoval::GridWallRemoval(double angle, double radius, double start_augmentation,
                                 double radius_augmentation, double fov, int max_points_per_cluster)
    : grid_geometry_(angle, radius, start_augmentation, radius_augmentation, fov),
      max_points_per_cluster_(max_points_per_cluster) {
  occlusion_bins_.resize(static_cast<int>(std::ceil(fov / angle)));
}

void GridWallRemoval::remove_walls(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
                                   sensor_msgs::msg::PointCloud2::SharedPtr& output_cloud) {
  // Initialize output cloud
  output_cloud->header = point_cloud->header;
  output_cloud->height = 1;
  output_cloud->is_bigendian = point_cloud->is_bigendian;
  output_cloud->point_step = point_cloud->point_step;
  output_cloud->fields = point_cloud->fields;
  output_cloud->width = 0;
  output_cloud->data.resize(point_cloud->width * point_cloud->point_step);

  std::fill(occlusion_bins_.begin(), occlusion_bins_.end(), std::numeric_limits<int>::max());

  const auto& cloud_data = point_cloud->data;
  const size_t num_points = point_cloud->width * point_cloud->height;
  if (num_points == 0) {
    return;
  }

  std::unordered_map<GridIndex, std::vector<size_t>> grid_map;

  for (size_t i = 0; i < num_points; ++i) {
    float x = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointX(i)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[LidarPoint::PointY(i)]);

    int slice = grid_geometry_.get_slice_index(x, y);
    int bin_idx = grid_geometry_.get_bin_index(x, y);

    if (slice >= 0 && slice < static_cast<int>(occlusion_bins_.size())) {
      grid_map[{slice, bin_idx}].push_back(i);
    }
  }

  std::unordered_map<GridIndex, bool> visited;
  for (auto& [start_cell, points] : grid_map) {
    if (visited[start_cell]) continue;

    std::queue<GridIndex> q;
    q.push(start_cell);
    visited[start_cell] = true;

    std::vector<GridIndex> cluster_cells;
    size_t cluster_pts = 0;

    while (!q.empty()) {
      GridIndex current = q.front();
      q.pop();
      cluster_cells.push_back(current);
      cluster_pts += grid_map[current].size();

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }

          GridIndex neighbor{current.x + dx, current.y + dy};
          if (grid_map.count(neighbor) && !visited[neighbor]) {
            visited[neighbor] = true;
            q.push(neighbor);
          }
        }
      }
    }

    // Determine if the cluster has to much points to be a cone
    if (static_cast<int>(cluster_pts) >= max_points_per_cluster_) {
      for (const auto& cell : cluster_cells) {
        // Save the front-most bin of the wall in each slice
        if (cell.y < occlusion_bins_[cell.x]) {
          occlusion_bins_[cell.x] = cell.y;
        }
      }
    }
  }

  // Eliminate bins after the occlusion bin in each slice and store the remaining points in output
  // cloud
  for (auto const& [cell, points] : grid_map) {
    if (cell.y < occlusion_bins_[cell.x]) {
      for (size_t idx : points) {
        size_t write_ptr = output_cloud->width * LidarPoint::POINT_STEP;
        std::memcpy(&output_cloud->data[write_ptr], &cloud_data[idx * LidarPoint::POINT_STEP],
                    LidarPoint::POINT_STEP);
        output_cloud->width++;
      }
    }
  }

  output_cloud->data.resize(output_cloud->width * LidarPoint::POINT_STEP);
  output_cloud->row_step = output_cloud->width * LidarPoint::POINT_STEP;
}