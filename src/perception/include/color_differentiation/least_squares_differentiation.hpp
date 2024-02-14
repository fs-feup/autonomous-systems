#include <color_differentiation/color_differentiation.hpp>


class LeastSquaresDifferentiation : ColorDifferentiation{
 public:
    Color colorDifferentiation(const pcl::PointCloud<pcl::PointXYZI>::Ptr
                               cone_point_cloud) const override;
};