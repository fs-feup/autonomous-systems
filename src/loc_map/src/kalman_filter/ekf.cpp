#include "kalman_filter/ekf.hpp"

#include "utils/position.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                           const Eigen::MatrixXd& C, const Eigen::MatrixXd& R,
                                           const Eigen::MatrixXd& Q)
    : A(A),
      B(B),
      C(C),
      R(R),
      Q(Q),
      expected_state(Eigen::ArrayXf::Zero(200)),
      P(Eigen::MatrixXf::Zero(200)){};

void ExtendedKalmanFilter::next_state(Eigen::VectorXd& state, Eigen::VectorXd& control,
                                      Eigen::VectorXd& measurement) {}