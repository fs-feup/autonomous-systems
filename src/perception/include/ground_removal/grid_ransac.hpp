#include <pcl/sample_consensus/ransac.h>

#include <string>
#include <utils/plane.hpp>

#include "ground_removal/ground_removal.hpp"
#include "ground_removal/ransac.hpp"

class GridRANSAC : public GroundRemoval {
 private:
  RANSAC ransac;
  int n_angular_grids;
  double radius_resolution;

 public:
  GridRANSAC(double epsilon, int n_tries, int n_angular_grids, double radius_resolution);

  void groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr ret, Plane& plane) const override;

                
  /**
   * @brief Get a sub point cloud from the input point cloud given two angles and two radii.
   *
   * This function extracts a sub point cloud from the input point cloud based on two angles
   * and two radius.
   *
   * @param[in] input_cloud The input point cloud.
   * @param[out] sub_cloud The sub point cloud extracted based on the specified angles and radii.
   * @param[in] start_angle The starting angle for the sub point cloud (in radians).
   * @param[in] end_angle The ending angle for the sub point cloud (in radians).
   * @param[in] min_radius The minimum radius for the sub point cloud.
   * @param[in] max_radius The maximum radius for the sub point cloud.
   */
  static void getSubPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr sub_cloud,
                        double start_angle, double end_angle,
                        double min_radius, double max_radius);

  static double getFarthestPoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
};