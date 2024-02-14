#include <cone_differentiation/cone_differentiation.hpp>


class LeastSquaresDifferentiation : ConeDifferentiation{
 public:
    Color coneDifferentiation(const pcl::PointCloud<pcl::PointXYZI>::Ptr
                               cone_point_cloud) const override;
};