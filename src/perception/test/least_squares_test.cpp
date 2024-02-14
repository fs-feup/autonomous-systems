#include <gtest/gtest.h>
#include "cone_differentiation/least_squares_differentiation.hpp"


class LeastSquaresDifferentiationTest : public ::testing::Test {
 protected:
    void SetUp() override {
        blue_cone.reset(new pcl::PointCloud<pcl::PointXYZI>);
        blue_cone->points.push_back(pcl::PointXYZI{1.0, 0.0, 0.0, 20});
        blue_cone->points.push_back(pcl::PointXYZI{0.0, 1.0, 1, 20});
        blue_cone->points.push_back(pcl::PointXYZI{0.0, 0.0, 2.0, 21});
        blue_cone->points.push_back(pcl::PointXYZI{0.0060, 0.0060, 4.0, 60.0});
        blue_cone->points.push_back(pcl::PointXYZI{10, 10, 5, 70});
        blue_cone->points.push_back(pcl::PointXYZI{10, 10, 4.5, 65});
        blue_cone->points.push_back(pcl::PointXYZI{10, 10, 7, 49});
        blue_cone->points.push_back(pcl::PointXYZI{10, 10, 9, 30});
        blue_cone->points.push_back(pcl::PointXYZI{10, 10, 10, 20});

        yellow_cone.reset(new pcl::PointCloud<pcl::PointXYZI>);

        yellow_cone->points.push_back(pcl::PointXYZI{1.0, 0.0, 0.0, 60});
        yellow_cone->points.push_back(pcl::PointXYZI{0.0, 1.0, 0, 70});
        yellow_cone->points.push_back(pcl::PointXYZI{0.0, 0.0, 2.0, 50});
        yellow_cone->points.push_back(pcl::PointXYZI{0.0060, 0.0060, 3.0, 50.0});
        yellow_cone->points.push_back(pcl::PointXYZI{10, 10, 4, 40});
        yellow_cone->points.push_back(pcl::PointXYZI{10, 10, 5, 20});
        yellow_cone->points.push_back(pcl::PointXYZI{10, 10, 6, 15});
        yellow_cone->points.push_back(pcl::PointXYZI{10, 10, 7, 30});
        yellow_cone->points.push_back(pcl::PointXYZI{10, 10, 8, 45});
        yellow_cone->points.push_back(pcl::PointXYZI{10, 10, 9, 60});
        yellow_cone->points.push_back(pcl::PointXYZI{10, 10, 10, 80});

        pcl_cloud_2_points.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl_cloud_2_points->points.push_back(pcl::PointXYZI{1.0, 0.0, 0.0, 0.5});
        pcl_cloud_2_points->points.push_back(pcl::PointXYZI{0.0, 1.0, 0.0, 1.0});
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr blue_cone;
    pcl::PointCloud<pcl::PointXYZI>::Ptr yellow_cone;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_2_points;
};

TEST_F(LeastSquaresDifferentiationTest, TestYellowCone) {
    auto cone_differentiator = new LeastSquaresDifferentiation();

    ASSERT_EQ(cone_differentiator->coneDifferentiation(yellow_cone), YELLOW);
}

TEST_F(LeastSquaresDifferentiationTest, TestBlueCone) {
    auto cone_differentiator = new LeastSquaresDifferentiation();

    ASSERT_EQ(cone_differentiator->coneDifferentiation(blue_cone), BLUE);
}

TEST_F(LeastSquaresDifferentiationTest, TestUndefinedCone) {
    auto cone_differentiator = new LeastSquaresDifferentiation();

    ASSERT_EQ(cone_differentiator->coneDifferentiation(pcl_cloud_2_points), UNDEFINED);
}