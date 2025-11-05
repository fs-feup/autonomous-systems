#include "clustering/grid_clustering.hpp"

GridClustering::GridClustering(double grid_width, int max_points_per_cluster,
                               int min_points_per_cluster, double big_grid_width,
                               int big_max_points_per_cluster)
    : grid_width_(grid_width),
      max_points_per_cluster_(max_points_per_cluster),
      min_points_per_cluster_(min_points_per_cluster),
      big_grid_width_(big_grid_width),
      big_max_points_per_cluster_(big_max_points_per_cluster) {}

void GridClustering::findBigClusterIndices(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud,
                                           std::vector<size_t>& big_cluster_indices) const {
  const auto& cloud_data = cloud->data;
  const size_t num_points = cloud->width * cloud->height;
  if (num_points == 0) return;

  std::unordered_map<GridIndex, std::vector<size_t>> grid_map;

  for (size_t i = 0; i < num_points; ++i) {
    float x = *reinterpret_cast<const float*>(&cloud_data[PointX(i)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[PointY(i)]);
    float z = *reinterpret_cast<const float*>(&cloud_data[PointZ(i)]);
    if (x == 0.0f && y == 0.0f && z == 0.0f) continue;

    int gx = static_cast<int>(std::floor(x / big_grid_width_));
    int gy = static_cast<int>(std::floor(y / big_grid_width_));
    grid_map[{gx, gy}].push_back(i);
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

    if (static_cast<int>(cluster_points.size()) < big_max_points_per_cluster_) {
      big_cluster_indices.insert(big_cluster_indices.end(), cluster_points.begin(),
                                 cluster_points.end());
    }
  }
}

void GridClustering::clustering(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                                std::vector<Cluster>* clusters) const {
  // Step 1: get indices from big clustering
  std::vector<size_t> selected_indices;
  findBigClusterIndices(input_cloud, selected_indices);

  if (selected_indices.empty()) return;

  const auto& cloud_data = input_cloud->data;

  // Step 2: grid clustering using small parameters but only over selected points
  std::unordered_map<GridIndex, std::vector<size_t>> grid_map;
  for (size_t i : selected_indices) {
    float x = *reinterpret_cast<const float*>(&cloud_data[PointX(i)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[PointY(i)]);
    float z = *reinterpret_cast<const float*>(&cloud_data[PointZ(i)]);
    if (x == 0.0f && y == 0.0f && z == 0.0f) continue;

    int gx = static_cast<int>(std::floor(x / grid_width_));
    int gy = static_cast<int>(std::floor(y / grid_width_));
    grid_map[{gx, gy}].push_back(i);
  }

  std::unordered_map<GridIndex, bool> visited;
  for (auto& [cell, points] : grid_map) {
    if (visited[cell]) continue;
    visited[cell] = true;

    std::queue<GridIndex> q;
    q.push(cell);
    std::vector<int> cluster_points;

    while (!q.empty()) {
      GridIndex current = q.front();
      q.pop();

      auto it = grid_map.find(current);
      if (it == grid_map.end()) continue;

      for (size_t idx : it->second) cluster_points.push_back(static_cast<int>(idx));

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

    if (static_cast<int>(cluster_points.size()) > min_points_per_cluster_ &&
        static_cast<int>(cluster_points.size()) < max_points_per_cluster_) {
      clusters->push_back(Cluster(input_cloud, cluster_points));
    }
  }
}
