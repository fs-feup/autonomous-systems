#include <cone_differentiation/cone_differentiation.hpp>

/**
 * @class LeastSquaresDifferentiation
 *
 * @brief Class for Cone Differentiation using the Least Squares Method.
 *
 */
class LeastSquaresDifferentiation : public ConeDifferentiation {
public:
  /**
   * @brief Perform cone differentiation on a cone's point cloud using the Least Squares Method.
   *
   * @param cone_point_cloud Cluster's point cloud.
   * @return Color The cone's color.
   */
  void coneDifferentiation(Cluster* cone_point_cloud) const override;
};
