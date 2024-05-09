#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace common_lib::maths {
void copy_eigen_sparse_matrix(const Eigen::SparseMatrix<float> &original,
                              Eigen::SparseMatrix<float> &copy);  // namespace maths
}  // namespace common_lib::maths