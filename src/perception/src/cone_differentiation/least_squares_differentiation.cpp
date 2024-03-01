#include <cone_differentiation/least_squares_differentiation.hpp>
#include <Eigen/Dense>

void LeastSquaresDifferentiation::coneDifferentiation(Cluster* cone_point_cloud) const {
    int n = cone_point_cloud->getPointCloud()->points.size();

    // It is impractical to compute a parabola using fewer than three points
    if (n < 3)  {
        cone_point_cloud->setColor("undefined");
        return;
    }


    // Solve x for: Ax = b
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd B(n);


    // Creation of matrix A and vector b
    for (int i = 0; i < n; ++i) {
        A(i, 0) = cone_point_cloud->getPointCloud()->points[i].z *
                  cone_point_cloud->getPointCloud()->points[i].z;
        A(i, 1) = cone_point_cloud->getPointCloud()->points[i].z;
        A(i, 2) = 1.0;
        B(i) = cone_point_cloud->getPointCloud()->points[i].intensity;
    }

    // Solve the linear system
    Eigen::Vector3d coefficients = A.colPivHouseholderQr().solve(B);

    cone_point_cloud->setColor(coefficients[0] > 0 ? "yellow" : "blue");
}