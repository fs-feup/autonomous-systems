#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>


/**
 * @brief Abstract base class for clustering point clouds.
 * 
 * The Clustering class defines an interface for clustering point clouds into groups or clusters.
 * Subclasses must implement the pure virtual function clustering().
 */
class Clustering {
 public:
      /**
       * @brief Clusters the input point cloud into groups.
       * 
       * This pure virtual function must be implemented by derived classes.
       * 
       * @param point_cloud A shared pointer to a point cloud of type pcl::PointCloud<pcl::PointXYZI>.
       * @param[out] clusters A pointer to a vector of shared pointers to point clouds to store the resulting clusters.
       */
       virtual void clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                           std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* clusters) const = 0;
};