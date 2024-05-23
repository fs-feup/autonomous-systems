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
#include <cmath>

GridRANSAC::GridRANSAC(double epsilon, int n_tries, int n_angular_grids, double radius_resolution) : 
        n_angular_grids(n_angular_grids), radius_resolution(radius_resolution) {

    this->ransac = RANSAC(epsilon, n_tries);
}

double GridRANSAC::get_furthest_point(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    double max_distance = 0.0;

    for (const auto& point : *cloud) {
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        if (distance > max_distance) {
            max_distance = distance;
        }
    }

    return max_distance;
}

void GridRANSAC::split_point_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                       std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>& grids) const {
    
    grids.clear();  // Clear any existing content in grids

    double max_distance = GridRANSAC::get_furthest_point(cloud);

    double angle_increment = 360.0 / n_angular_grids;

    // Matrix Initialization
    for (int radius = 0; radius < max_distance; radius += radius_resolution) {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> row;
        for (int i = 0; i < n_angular_grids; i++) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_row(new pcl::PointCloud<pcl::PointXYZI>());
            row.push_back(pc_row);
        }
        grids.push_back(row);
    }

    // Matrix Population
    for (const auto& point : *cloud) {
        double radius = std::sqrt(point.x * point.x + point.y * point.y);
        double angle = std::atan2(point.y, point.x) * 180.0 / M_PI;  // Convert to degrees

        // Normalize angle to be within [0, 360)
        if (angle < 0) {
            angle += 360.0;
        }

        int row = static_cast<int>(radius / radius_resolution);
        int column = static_cast<int>(angle / angle_increment) % n_angular_grids;

        if (row < grids.size()) {
            grids[row][column]->points.push_back(point);
        }
    }
}


void GridRANSAC::groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr ret, Plane& plane) const {
    
    // reseting everything

    ret->clear();
    
    plane = Plane(0,0,0,0);

    
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> grids;
    split_point_cloud(point_cloud, grids); // Get the grids organized in a matrix format

    int count = 0;
    
    // Multithreading using OpenMP
    #pragma omp parallel for reduction(+:count) schedule(dynamic)
    for (int i = 0; i < grids.size(); ++i){
        for (int j = 0; j < grids[i].size(); ++j){
            Plane grid_plane;
            pcl::PointCloud<pcl::PointXYZI>::Ptr grid_ret(new pcl::PointCloud<pcl::PointXYZI>);
            this->ransac.groundRemoval(grids[i][j], grid_ret, grid_plane);
            #pragma omp critical
            {
                *ret += *grid_ret;
                plane += grid_plane;
                count++;
            }
            
        }
    }

    plane /= count;
}