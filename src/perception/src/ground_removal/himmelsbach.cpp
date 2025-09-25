#include "ground_removal/himmelsbach.hpp"

Himmelsbach::Himmelsbach(const double max_slope) : max_slope(max_slope) {}

void Himmelsbach::ground_removal(const pcl::PointCloud<PointXYZIR>::Ptr point_cloud,
                                 const pcl::PointCloud<PointXYZIR>::Ptr ret, Plane& plane,
                                 [[maybe_unused]] const SplitParameters split_params) const {
  // Slip the point cloud into slices, each slice contains several rings
  // std::vector<Slice> splited_cloud;
  // split_point_cloud(point_cloud, splited_cloud, split_params);

  // for (int i = 0; i < static_cast<int>(splited_cloud.size()); ++i) {
  //   for (int j = 0; j < static_cast<int>(splited_cloud[i].rings.size()); ++j) {
  //     RCLCPP_INFO(rclcpp::get_logger("Himmelsbach"), "Slice %d, Ring %d, Number of points: %d",
  //     i,
  //                 j, static_cast<int>(splited_cloud[i].rings[j].indices.size()));
  //   }
  // }

  ret->points.clear();
  ret->width = 0;
  ret->height = 1;
  ret->is_dense = true;

  int n = 30000;
  for (auto point : point_cloud->points) {
    if (n >= 0) {
      ret->points.push_back(point);
      ret->width++;
      n--;
    }
  }
}

void Himmelsbach::split_point_cloud(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                                    std::vector<Slice>& splited_cloud,
                                    const SplitParameters split_params) const {
  int n_slices =
      static_cast<int>(std::ceil(split_params.fov_angle / split_params.angle_resolution));
  int n_points_per_ring = static_cast<int>(
      std::ceil(split_params.angle_resolution / split_params.lidar_horizontal_resolution));
  splited_cloud.resize(n_slices);

  for (auto& slice : splited_cloud) {
    slice.rings.resize(40);  // max rings
    for (auto& ring : slice.rings) {
      ring.indices.reserve(n_points_per_ring);
    }
  }

  int slice_idx = 0;
  int prev_ring = -1;
  int lines_for_next_slice = n_points_per_ring;
  int slice_count = 0;

  for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
    const auto& pt = cloud->points[i];
    splited_cloud[slice_idx].rings[pt.ring].indices.push_back(i);
    slice_count++;

    if (pt.ring == 32) {
      // RCLCPP_INFO(rclcpp::get_logger("Himmelsbach"),
      //            "Point: ring %d, slice %d, lines for next slice %d, remaining points: %d",
      //            pt.ring, slice_idx, lines_for_next_slice,
      //            static_cast<int>(cloud->points.size()) - i);
    }
    if (pt.ring < prev_ring) {
      lines_for_next_slice--;
    }

    if (lines_for_next_slice <= 0) {
      slice_idx++;
      slice_count = 0;
      lines_for_next_slice = n_points_per_ring;
      if (slice_idx >= n_slices) {
        // RCLCPP_INFO(rclcpp::get_logger("Himmelsbach"),
        //            "All slices done------------------------------------------.");
        break;  // stop if done
      }
    }
    prev_ring = pt.ring;
  }
}
