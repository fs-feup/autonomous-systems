#include <cone_validator/cylinder_validator.hpp>

bool CylinderValidator::coneValidator(Cluster* cone_point_cloud) const {

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud = cone_point_cloud->getPointCloud();

    
    for (const auto& point : *cone_point_cloud->getPointCloud()) {

        double radius = 0.1612;
        double height = 0.325;
        
        double distanceXY = std::sqrt((point.x - cone_point_cloud->getCentroid().x()) * (point.x - cone_point_cloud->getCentroid().x()) +
                                       (point.y - cone_point_cloud->getCentroid().y()) * (point.y - cone_point_cloud->getCentroid().y()));
        // Calculate distance between the point and the centroid along the z-axis
        
        double distanceZ = std::abs(point.z - cone_point_cloud->getCentroid().z());
        
        // If the point is outside the cylinder, return false
        if (distanceXY > radius || distanceZ > height / 2)
            return false;
             
    }
    
    // All points are on the surface of the cylinder
    return true;
}