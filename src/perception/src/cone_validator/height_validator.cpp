#include <cone_validator/height_validator.hpp>

HeightValidator::HeightValidator(double height) : height(height) {}

bool HeightValidator::coneValidator(Cluster* cone_point_cloud, Plane& plane) const {
    double maxZ = cone_point_cloud->getPointCloud()->points[0].z;
    auto maxPoint = cone_point_cloud->getPointCloud()->points[0];

    for (long unsigned int i = 0; i < cone_point_cloud->getPointCloud()->points.size(); i++) {
        if (cone_point_cloud->getPointCloud()->points[i].z > maxZ) {
            maxZ = cone_point_cloud->getPointCloud()->points[i].z;
            maxPoint = cone_point_cloud->getPointCloud()->points[i];
        }
    }

    return plane.getDistanceToPoint(maxPoint) < height;
}