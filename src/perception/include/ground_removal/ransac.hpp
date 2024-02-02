#include "ground_removal.hpp"
#include <string>

class RANSAC : public GroundRemoval {
private:
    double epsilon;
    int n_tries;

public:
    RANSAC(double epsilon, int n_tries);
    sensor_msgs::msg::PointCloud2 groundRemoval(sensor_msgs::msg::PointCloud2 point_cloud) const override;
};