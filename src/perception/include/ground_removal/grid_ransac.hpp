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

  static double getFarthestPoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  void split_point_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                                std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>& grids) const;
};