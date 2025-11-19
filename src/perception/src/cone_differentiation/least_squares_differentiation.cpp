#include <Eigen/Dense>
#include <cone_differentiation/least_squares_differentiation.hpp>

void LeastSquaresDifferentiation::coneDifferentiation(Cluster* cone_point_cloud) const {
  int n = cone_point_cloud->get_point_indices().size();

  // It is impractical to compute a parabola using fewer than three points
  if (n < 3) {
    cone_point_cloud->set_color("undefined");
    return;
  }

  // Solve x for: Ax = b
  Eigen::MatrixXd A(n, 3);
  Eigen::VectorXd B(n);

  // Creation of matrix A and vector b
  for (int i = 0; i < n; ++i) {
    float z = *reinterpret_cast<const float*>(
        &cone_point_cloud->get_point_cloud()
             ->data[LidarPoint::PointZ(cone_point_cloud->get_point_indices()[i])]);
    float intensity = *reinterpret_cast<const float*>(
        &cone_point_cloud->get_point_cloud()
             ->data[LidarPoint::PointIntensity(cone_point_cloud->get_point_indices()[i])]);
    A(i, 0) = z * z;
    A(i, 1) = z;
    A(i, 2) = 1.0;
    B(i) = intensity;
  }

  // Solve the linear system
  Eigen::Vector3d coefficients = A.colPivHouseholderQr().solve(B);

  if (coefficients[0] > 0) {
    cone_point_cloud->set_color("yellow");
  } else {
    cone_point_cloud->set_color("blue");
  }
}