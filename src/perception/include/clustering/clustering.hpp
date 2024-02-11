#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

class Clustering {
 public:
       virtual void clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                           std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* clusters) const = 0;
};