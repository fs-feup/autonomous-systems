#include "clustering/dbscan.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

DBSCAN::DBSCAN(int min_cluster_size, double neighbours_dist_threshold)
    : min_cluster_size(min_cluster_size), neighbours_dist_threshold(neighbours_dist_threshold) {}

void DBSCAN::clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                        std::vector<Cluster>* clusters) const {
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(point_cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(neighbours_dist_threshold);
  ec.setMinClusterSize(min_cluster_size);

  ec.setSearchMethod(tree);
  ec.setInputCloud(point_cloud);

  // PointIndices - A defined PCL struct - vector of indices of cluster's points
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);

  // Iterate over clusters to get the values of the points
  for (const auto& indices : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
    for (const int& index : indices.indices) {
      cluster->points.push_back(point_cloud->points[index]);
    }
    cluster->width = cluster->size();
    cluster->height = 1;
    cluster->is_dense = true;
    clusters->push_back(Cluster(cluster));
  }
}
