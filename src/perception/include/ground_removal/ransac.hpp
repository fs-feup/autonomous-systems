#include "ground_removal.hpp"
#include <pcl/sample_consensus/ransac.h>
#include <string>

class RANSAC : public GroundRemoval {
private:
    double epsilon;
    int n_tries;

public:
    RANSAC(double epsilon, int n_tries);
    void groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, pcl::PointCloud<pcl::PointXYZI> &ret) const override;
};