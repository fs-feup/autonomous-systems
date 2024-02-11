#include <gtest/gtest.h>
#include "clustering/dbscan.hpp"

class DBSCANTest : public ::testing::Test {
 protected:
    void SetUp() override {
        point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

        point_cloud->push_back(pcl::PointXYZI(1.0, 2.0, 0.0, 0.1));
        point_cloud->push_back(pcl::PointXYZI(4.0, 5.0, 0.0, 0.2));
        point_cloud->push_back(pcl::PointXYZI(7.0, 8.0, 0.0, 0.3));
        point_cloud->push_back(pcl::PointXYZI(4.0, 3.5, 0.0, 0.4));
        point_cloud->push_back(pcl::PointXYZI(0, 7.0, 0.0, 0.5));
        point_cloud->push_back(pcl::PointXYZI(4.0, 4.0, 0.0, 0.6));
        point_cloud->push_back(pcl::PointXYZI(5.0, 4.5, 0.0, 0.7));
        point_cloud->push_back(pcl::PointXYZI(40.0, 40.0, 0.0, 0.8));
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud;
};


TEST_F(DBSCANTest, Test1) {
    auto clustering = new DBSCAN(0, 0);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 8);
}



TEST_F(DBSCANTest, Test2) {
    auto clustering = new DBSCAN(0, 30);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 2);
}


TEST_F(DBSCANTest, Test3) {
    auto clustering = new DBSCAN(2, 30);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 1);
}



TEST_F(DBSCANTest, Test4) {
    auto clustering = new DBSCAN(0, 50);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 1);
}

TEST_F(DBSCANTest, Test5) {
    auto clustering = new DBSCAN(0, 1.5);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 5);
}

TEST_F(DBSCANTest, Test6) {
    auto clustering = new DBSCAN(2, 1.5);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 1);
}

TEST_F(DBSCANTest, Test7) {
    auto clustering = new DBSCAN(0, 0.4);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 8);
}

TEST_F(DBSCANTest, Test8) {
    auto clustering = new DBSCAN(0, 0.6);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 7);
}

TEST_F(DBSCANTest, Test9) {
    auto clustering = new DBSCAN(10, 50);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 0);
}


TEST_F(DBSCANTest, Test10) {
    auto clustering = new DBSCAN(1, 0.1);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    clustering->clustering(point_cloud, &clusters);

    ASSERT_EQ(clusters.size(), 8);
}

