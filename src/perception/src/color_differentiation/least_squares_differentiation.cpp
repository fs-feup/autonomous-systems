#include <color_differentiation/least_squares_differentiation.hpp>
#include <Eigen/Dense>

Color LeastSquaresDifferentiation::colorDifferentiation(const pcl::PointCloud<pcl::PointXYZI>::Ptr
                                                        cone_point_cloud) const {
    int n = cone_point_cloud->points.size();
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd B(n);

    for (int i = 0; i < n; ++i) {
        A(i, 0) = cone_point_cloud->points[i].z * cone_point_cloud->points[i].z;
        A(i, 1) = cone_point_cloud->points[i].z;
        A(i, 2) = 1.0;
        B(i) = cone_point_cloud->points[i].intensity;
    }

    Eigen::Vector3d coefficients = A.colPivHouseholderQr().solve(B);

    return coefficients[0] > 0 ? YELLOW : BLUE;
}