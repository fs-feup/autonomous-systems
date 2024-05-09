#include "common_lib/maths/matrixes.hpp"

namespace common_lib::maths {
void copy_eigen_sparse_matrix(Eigen::SparseMatrix<float> &original,
                              Eigen::SparseMatrix<float> &copy) {
  for (int i = 0; i < original.rows(); i++) {
    for (int j = 0; j < original.cols(); j++) {
      copy.coeffRef(i, j) = original.coeff(i, j);
    }
  }
}
}  // namespace common_lib