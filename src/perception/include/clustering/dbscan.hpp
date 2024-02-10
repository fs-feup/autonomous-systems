#include "clustering/clustering.hpp"
#include <string>


class DBSCAN : public Clustering {
 private:
    double n_neighbours;
    int dist_threshold;
    int min_cluster_size;

 public:

    DBSCAN(double epsilon, int n_tries);

    void clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr ret) const override;
};