#include <gtest/gtest.h>
#include "cone_differentiation/least_squares_differentiation.hpp"
#include <utils/cluster.hpp>


/**
 * @brief Fixture for testing the LeastSquaresDifferentiation class.
 */
class LeastSquaresDifferentiationTest : public ::testing::Test {
 protected:
    /**
     * @brief Set up the test fixtures.
     */
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

        real_blue_cone.reset(new pcl::PointCloud<pcl::PointXYZI>);

        real_blue_cone->points.push_back(pcl::PointXYZI{1.0, 0.0, 0.3, 5});
        real_blue_cone->points.push_back(pcl::PointXYZI{1.0, 0.0, 0.3, 1});
        real_blue_cone->points.push_back(pcl::PointXYZI{1.0, 0.0, 0.3, 72});
        real_blue_cone->points.push_back(pcl::PointXYZI{1.0, 0.0, 0.27, 6});
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr blue_cone;
    pcl::PointCloud<pcl::PointXYZI>::Ptr yellow_cone;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_2_points;
    pcl::PointCloud<pcl::PointXYZI>::Ptr real_blue_cone;
};


/**
 * @brief Test case to validate the cone differentiation for a yellow cone.
 */
TEST_F(LeastSquaresDifferentiationTest, TestYellowCone) {
    Cluster* cluster = new Cluster(yellow_cone);
    auto cone_differentiator = new LeastSquaresDifferentiation();

    cone_differentiator->coneDifferentiation(cluster);

    ASSERT_EQ(cluster->getColor(), "yellow");
}

/**
 * @brief Test case to validate the cone differentiation for a blue cone.
 */
TEST_F(LeastSquaresDifferentiationTest, TestBlueCone) {
    Cluster* cluster = new Cluster(blue_cone);
    auto cone_differentiator = new LeastSquaresDifferentiation();

    cone_differentiator->coneDifferentiation(cluster);

    ASSERT_EQ(cluster->getColor(), "blue");
}


/**
 * @brief Test case to validate the cone differentiation for an undefined cone.
 */
TEST_F(LeastSquaresDifferentiationTest, TestUndefinedCone) {
    Cluster* cluster = new Cluster(pcl_cloud_2_points);

    auto cone_differentiator = new LeastSquaresDifferentiation();

    cone_differentiator->coneDifferentiation(cluster);

    ASSERT_EQ(cluster->getColor(), "undefined");
}


