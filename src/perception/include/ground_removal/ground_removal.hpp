#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "sensor_msgs/msg/point_cloud2.hpp"

/**
 * @class GroundRemoval
 * @brief Abstract class for ground removal from a point cloud.
 *
 * This class defines an interface for ground removal algorithms from a point cloud.
 * Inherit from this class to implement specific ground removal algorithms.
 */
class GroundRemoval {
 public:
  /**
   * @brief Perform ground removal on the input point cloud.
   *
   * This pure virtual function must be implemented by derived classes.
   *
   * @param point_cloud The input point cloud to be processed.
   * @param[out] ret The resulting point cloud after ground removal.
   */
  virtual void groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr ret, Plane& plane) const = 0;
};