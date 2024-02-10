#include "clustering/clustering.hpp"
#include <string>


class DBSCAN : public Clustering {
 private:
    int n_neighbours;
    double neighbours_dist_threshold;
    double dist_threshold;
    int min_cluster_size;

 public:

    DBSCAN(int n_neighbours, double neighbours_dist_threshold,
           double dist_threshold, int min_cluster_size);

    void clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters) const override;
};