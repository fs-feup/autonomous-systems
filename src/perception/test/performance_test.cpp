#include <gtest/gtest.h>
#include "ground_removal/ransac.hpp"
#include "clustering/dbscan.hpp"
#include "cone_differentiation/least_squares_differentiation.hpp"
#include <chrono>
#include <sstream>
#include <ctime>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <pcl_conversions/pcl_conversions.h>

/**
 * @class PerceptionPerformanceTest
 * @brief Test fixture for performance testing of perception algorithms.
 */
class PerceptionPerformanceTest : public ::testing::Test {
protected:
    /**
     * @brief Set up the test fixture.
     */
    void SetUp() override {
        pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        ground_removal = new RANSAC(RANSAC_eps, RANSAC_Iter);
        clustering = new DBSCAN(DBSCAN_neighbours_threshold, DBSCAN_dist_threshold);
        cone_differentiator = new LeastSquaresDifferentiation();
    }

    /**
     * @brief Get current date and time as a string.
     * @return Current date and time as a string in "YYYY-MM-DD-HH:MM" format.
     */
    std::string getCurrentDateTimeAsString() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm* now_tm = std::localtime(&now_time);
        std::stringstream ss;
        ss << std::put_time(now_tm, "%Y-%m-%d-%H:%M");
        return ss.str();
    }

    /**
     * @brief Write performance statistics to a CSV file.
     * @param initial_n_points Initial number of points in the point cloud.
     * @param RANSAC_time Duration of RANSAC algorithm in milliseconds.
     * @param ground_removed_size Number of points after ground removal.
     * @param DBSCAN_time Duration of DBSCAN algorithm in milliseconds.
     * @param ConeDif_time Duration of cone differentiation algorithm in milliseconds.
     * @param Total_time Total duration of the test in milliseconds.
     * @param generatedClusters Number of clusters generated.
     * @param blues Number of blue cones detected.
     * @param yellows Number of yellow cones detected.
     * @param undefineds Number of cones with undefined color detected.
     * @param conversion_duration Duration of point cloud conversion in milliseconds.
     */
    void writeToFile(long long initial_n_points,
                     long long RANSAC_time,
                     long long ground_removed_size,
                     long long DBSCAN_time,
                     long long ConeDif_time,
                     long long Total_time,
                     long long generatedClusters,
                     long long blues,
                     long long yellows,
                     long long undefineds,
                     long long conversion_duration) {
        bool fileExists = std::filesystem::exists(output_statistics_path_file);

        std::ofstream csv_file(output_statistics_path_file, std::ios::app);
        if (!fileExists) {
            csv_file << "Initial Number of Points,Conversion Duration,"
                        << "RANSAC Epsilon,RANSAC Number Iterations,"
                        << "Number of Points after RANSAC,RANSAC Duration(ms),"
                        << "DBSCAN Distance Threshold,DBSCAN Neighbours Threshold,"
                        << "Number of Generated Clusters,"
                        << "DBSCAN Duration(ms),"
                        << "Number of Blue Cones,Number of Yellow Cones,"
                        << "Number of Undefined Cones,ConeDifferentiation Duration(ms),"
                        << "Total Duration(ms)\n";
        }

        csv_file << std::fixed << std::setprecision(6);
        csv_file << initial_n_points << ","
            << std::fixed << std::setprecision(3) << conversion_duration << ","
            << RANSAC_eps << ","
            << RANSAC_Iter << ","
            << ground_removed_size << ","
            << std::fixed << std::setprecision(3) << RANSAC_time << ","
            << DBSCAN_dist_threshold << ","
            << DBSCAN_neighbours_threshold << ","
            << generatedClusters << ","
            << std::fixed << std::setprecision(3) << DBSCAN_time << ","
            << blues << ","
            << yellows << ","
            << undefineds << ","
            << std::fixed << std::setprecision(3) << ConeDif_time << ","
            << std::fixed << std::setprecision(3) << Total_time << std::endl;

        csv_file.close();
    }

protected:
    std::string output_statistics_path_file = "../../performance/exec_time/perception_" +
                    getCurrentDateTimeAsString() + ".csv";
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

/**
 * @brief Test case for performance testing of perception algorithms.
 */
TEST_F(PerceptionPerformanceTest, TestPerformance) {
    for (long unsigned int it = 0; it <= 7; it++) {
        std::string file_name =  inputFilesPaths + "PointCloud" + std::to_string(it) + ".pcd";

        // Point cloud loading and transform into ROS msg
        pcl::io::loadPCDFile<pcl::PointXYZI>(file_name, *pcl_cloud);
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*pcl_cloud, msg);
        pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

        auto start_time = std::chrono::high_resolution_clock::now();

        pcl::fromROSMsg(msg, *pcl_cloud);

        auto conversion_time = std::chrono::high_resolution_clock::now();

        auto conversion_duration = std::chrono::duration_cast<std::chrono::milliseconds>
            (start_time - conversion_time);

        pcl::PointCloud<pcl::PointXYZI>::Ptr
            ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        Plane plane;

        ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

        auto ransac_time = std::chrono::high_resolution_clock::now();
        auto ransac_duration = std::chrono::duration_cast<std::chrono::milliseconds>
            (ransac_time - conversion_time);

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

        auto total_time = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>
                (end_time - start_time);
        writeToFile(pcl_cloud->size(), ransac_duration.count(), ground_removed_cloud->size(),
                dbscan_duration.count(), coneDifferentiaion_duration.count(),
                total_time.count(), clusters.size(), blues, yellows, undefineds,
                        conversion_duration.count());
    }
}
