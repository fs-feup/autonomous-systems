#include "ground_removal/ransac.hpp"
#include "ground_removal/grid_ransac.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <utils/plane.hpp>

GridRANSAC::GridRANSAC(double epsilon, int n_tries, int n_angular_grids, double radius_resolution) : 
        n_angular_grids(n_angular_grids), radius_resolution(radius_resolution) {

    this->ransac = RANSAC(epsilon, n_tries);
}

void GridRANSAC::getSubPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr sub_cloud,
                        double start_angle, double end_angle,
                        double min_radius, double max_radius) {
    sub_cloud->clear();

    for (const auto& point : *input_cloud) {
      double radius = std::sqrt(point.x * point.x + point.y * point.y);
      double angle = std::atan2(point.y, point.x);
      if (radius >= min_radius && radius <= max_radius &&
          angle >= start_angle && angle <= end_angle) {
        sub_cloud->push_back(point);
      }
    }
}

double GridRANSAC::getFarthestPoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    double max_distance = 0.0;

    for (const auto& point : *cloud) {
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        if (distance > max_distance) {
            max_distance = distance;
        }
    }

    return max_distance;
}

void GridRANSAC::groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr ret, Plane& plane) const {

    ret->clear();

    double angle_increment = 360.0 / n_angular_grids;

    double max_distance = GridRANSAC::getFarthestPoint(point_cloud);

    int count = 0;

    // Add multithreading here
    for (int ang = 0; ang < 360; ang += angle_increment){
        for (int radius = 0; radius < max_distance; radius += radius_resolution){
            pcl::PointCloud<pcl::PointXYZI>::Ptr sub_point_cloud;
            GridRANSAC::getSubPointCloud(point_cloud, sub_point_cloud, ang, 
                                         ang + angle_increment, radius, radius + radius_resolution);
            
            Plane grid_plane;
            pcl::PointCloud<pcl::PointXYZI>::Ptr grid_ret;
            this->ransac.groundRemoval(point_cloud, grid_ret, grid_plane);

            *ret = *ret + *grid_ret;

            plane += grid_plane;
            count++;
        }
    }

    plane /= count;
}