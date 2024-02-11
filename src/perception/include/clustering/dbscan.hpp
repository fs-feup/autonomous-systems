#include "clustering/clustering.hpp"
#include <string>
#include <vector>

class DBSCAN : public Clustering {
 private:
    int min_cluster_size;
    double neighbours_dist_threshold;

 public:
    DBSCAN(int min_cluster_size, double neighbours_dist_threshold);

    void clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* clusters) const override;
};