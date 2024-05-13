#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "clustering/dbscan.hpp"
#include "cone_differentiation/least_squares_differentiation.hpp"
#include "ground_removal/ransac.hpp"

/**
 * @struct PerceptionExecutionData
 * @brief Struct to store data related to perception execution.
 */
struct PerceptionExecutionData {
  long long initial_n_points;     ///< Initial number of points.
  long long RANSAC_time;          ///< Time taken by RANSAC algorithm.
  long long ground_removed_size;  ///< Size of ground removed.
  long long DBSCAN_time;          ///< Time taken by DBSCAN algorithm.
  long long ConeDif_time;         ///< Time taken by ConeDif algorithm.
  long long Total_time;           ///< Total time taken.
  long long generatedClusters;    ///< Number of clusters generated.
  long long blues;                ///< Number of points identified as blue.
  long long yellows;              ///< Number of points identified as yellow.
  long long undefineds;           ///< Number of points identified as undefined.
  long long conversion_duration;  ///< Duration of conversion.
  double RANSAC_eps;              ///< RANSAC epsilon value.
};

/**
 * @class PerceptionPerformanceTest
 * @brief Test fixture for performance testing of perception algorithms.
 */
class PerceptionPerformanceTest : public ::testing::Test {
 public:
  /**
   * @brief Set up the test fixture.
   */
  void SetUp() override {
    pcl_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
    ground_removal = std::make_unique<RANSAC>(RANSAC_eps, RANSAC_Iter);
    clustering = std::make_unique<DBSCAN>(DBSCAN_neighbours_threshold, DBSCAN_dist_threshold);
    cone_differentiator = std::make_unique<LeastSquaresDifferentiation>();
  }

  /**
   * @brief Get current date and time as a string.
   * @return Current date and time as a string in "YYYY-MM-DD-HH:MM" format.
   */
  std::string getCurrentDateTimeAsString() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm;
    localtime_r(&now_time, &now_tm);

    std::stringstream ss;
    ss << std::put_time(&now_tm, "%Y-%m-%d-%H:%M");
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
  void writeToFile(const PerceptionExecutionData& executionTime) {
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
    csv_file << executionTime.initial_n_points << "," << std::fixed << std::setprecision(3)
             << executionTime.conversion_duration << "," << RANSAC_eps << "," << RANSAC_Iter << ","
             << executionTime.ground_removed_size << "," << std::fixed << std::setprecision(3)
             << executionTime.RANSAC_time << "," << DBSCAN_dist_threshold << ","
             << executionTime.DBSCAN_time << "," << executionTime.generatedClusters << ","
             << std::fixed << std::setprecision(3) << executionTime.DBSCAN_time << ","
             << executionTime.blues << "," << executionTime.yellows << ","
             << executionTime.undefineds << "," << std::fixed << std::setprecision(3)
             << executionTime.ConeDif_time << "," << std::fixed << std::setprecision(3)
             << executionTime.Total_time << std::endl;

    csv_file.close();
  }
  std::string output_statistics_path_file =
      "../../performance/exec_time/perception/perception_" + getCurrentDateTimeAsString() + ".csv";
  std::string inputFilesPaths = "../../src/perception/test/point_clouds/";

  std::unique_ptr<GroundRemoval> ground_removal;
  std::unique_ptr<Clustering> clustering;
  std::unique_ptr<ConeDifferentiation> cone_differentiator;
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
    std::stringstream ss;
    ss << inputFilesPaths << "PointCloud" << it << ".pcd";
    std::string file_name = ss.str();

    struct PerceptionExecutionData executionTime;

    // Point cloud loading and transform into ROS msg
    pcl::io::loadPCDFile<pcl::PointXYZI>(file_name, *pcl_cloud);
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*pcl_cloud, msg);
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

    auto start_time = std::chrono::high_resolution_clock::now();

    pcl::fromROSMsg(msg, *pcl_cloud);

    auto conversion_time = std::chrono::high_resolution_clock::now();

    executionTime.conversion_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(start_time - conversion_time).count();

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    Plane plane;

    ground_removal->groundRemoval(pcl_cloud, ground_removed_cloud, plane);

    auto ransac_time = std::chrono::high_resolution_clock::now();
    executionTime.RANSAC_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(ransac_time - conversion_time)
            .count();

    std::vector<Cluster> clusters;
    clustering->clustering(ground_removed_cloud, &clusters);

    auto dbscan_time = std::chrono::high_resolution_clock::now();
    executionTime.DBSCAN_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(dbscan_time - ransac_time).count();

    executionTime.blues = 0;
    executionTime.yellows = 0;
    executionTime.undefineds = 0;
    for (auto cluster : clusters) {
      cone_differentiator->coneDifferentiation(&cluster);
      std::string color = cluster.getColor();
      if (color == "blue")
        executionTime.blues++;
      else if (color == "yellow")
        executionTime.yellows++;
      else
        executionTime.undefineds++;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    executionTime.ConeDif_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - dbscan_time).count();

    executionTime.Total_time =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_time - start_time)
            .count();

    executionTime.initial_n_points = pcl_cloud->points.size();
    executionTime.ground_removed_size = ground_removed_cloud->points.size();

    writeToFile(executionTime);
  }
}
