#include <gtest/gtest.h>
#include "ground_removal/ransac.hpp"
#include "clustering/dbscan.hpp"
#include "cone_differentiation/least_squares_differentiation.hpp"
#include <chrono>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <filesystem>


class PerceptionPerformanceTest : public ::testing::Test {
 protected:
    void SetUp() override {
        pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        ground_removal = new RANSAC(RANSAC_eps, RANSAC_Iter);
        clustering = new DBSCAN(DBSCAN_neighbours_threshold, DBSCAN_dist_threshold);
        cone_differentiator = new LeastSquaresDifferentiation();
    }

    void writeToFile(long long initial_n_points,
                     long long RANSAC_time,
                     long long ground_removed_size,
                     long long DBSCAN_time,
                     long long ConeDif_time,
                     long long Total_time,
                     long long generatedClusters,
                     long long blues,
                     long long yellows,
                     long long undefineds) {
        bool fileExists = std::filesystem::exists(output_statistics_path_file);

        std::ofstream csv_file(output_statistics_path_file, std::ios::app);
        if (!fileExists) {
            csv_file << "Initial Number of Points,RANSAC Epsilon,RANSAC Number Iterations,"
                        << "Number of Points after RANSAC,RANSAC Duration(ms),"
                        << "DBSCAN Distance Threshold,DBSCAN Neighbours Threshold,"
                        << "Number of Generated Clusters,"
                        << "DBSCAN Duration(ms),"
                        << "Number of Blue Cones,Number of Yellow Cones,"
                        << "Number of Undefined Cones,ConeDifferentiation Duration(ms),"
                        << "Total Duration(ms)\n";
        }

        csv_file << initial_n_points << ","
            << RANSAC_eps << ","
            << RANSAC_Iter << ","
            << ground_removed_size << ","
            << RANSAC_time << ","
            << DBSCAN_dist_threshold << ","
            << DBSCAN_neighbours_threshold << ","
            << generatedClusters << ","
            << DBSCAN_time << ","
            << blues << ","
            << yellows << ","
            << undefineds << ","
            << ConeDif_time << ","
            << Total_time << std::endl;

        csv_file.close();
    }


    std::string output_statistics_path_file = "../../src/performance/exec_time/perception02.csv";
    std::string inputFilesPaths = "../../src/perception/test/point_clouds/";

    GroundRemoval* ground_removal;
    Clustering* clustering;
    ConeDifferentiation* cone_differentiator;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;

    float RANSAC_eps = 0.1;
    int RANSAC_Iter = 20;

    float DBSCAN_dist_threshold = 0.1;
    int DBSCAN_neighbours_threshold = 1;
};



TEST_F(PerceptionPerformanceTest, TestPerformance) {
    for (int it = 0; it <= 106; it++) {
        std::string file_name =  inputFilesPaths + "PointCloud" + std::to_string(it) + ".pcd";

        pcl::io::loadPCDFile<pcl::PointXYZI>(file_name, *pcl_cloud);

        auto start_time = std::chrono::high_resolution_clock::now();

        pcl::PointCloud<pcl::PointXYZI>::Ptr
            ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud);

        auto ransac_time = std::chrono::high_resolution_clock::now();
        auto ransac_duration = std::chrono::duration_cast<std::chrono::milliseconds>
            (ransac_time - start_time);

        std::vector<Cluster> clusters;
        clustering->clustering(ground_removed_cloud, &clusters);

        auto dbscan_time = std::chrono::high_resolution_clock::now();
        auto dbscan_duration = std::chrono::duration_cast<std::chrono::milliseconds>
            (dbscan_time - ransac_time);


        unsigned int blues = 0;
        unsigned int yellows = 0;
        unsigned int undefineds = 0;
        for (int i = 0; i < clusters.size(); i++) {
            cone_differentiator->coneDifferentiation(&clusters[i]);
            std::string color = clusters[i].getColor();
            if (color == "blue") blues++;
            else if (color == "yellow") yellows++;
            else
                undefineds++;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto coneDifferentiaion_duration = std::chrono::duration_cast<std::chrono::milliseconds>
            (end_time - dbscan_time);

        auto total_time = std::chrono::duration_cast
            <std::chrono::milliseconds>(end_time - start_time);
        writeToFile(pcl_cloud->size(), ransac_duration.count(), ground_removed_cloud->size(),
                dbscan_duration.count(), coneDifferentiaion_duration.count(),
                total_time.count(), clusters.size(), blues, yellows, undefineds);
    }
}
