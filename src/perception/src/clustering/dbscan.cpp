#include "clustering/dbscan.hpp"

DBSCAN::DBSCAN(int min_cluster_size, double neighbours_dist_threshold)
    : min_cluster_size(min_cluster_size), neighbours_dist_threshold(neighbours_dist_threshold) {}

void DBSCAN::clustering(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                        std::vector<Cluster>* clusters) const {
  // Missing implementation of DBSCAN clustering algorithm without using PCL.
  (void)input_cloud; // To avoid unused parameter warning
  (void)clusters;
}
