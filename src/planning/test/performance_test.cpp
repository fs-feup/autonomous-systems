#include <chrono>
#include <fstream>
#include <set>
#include "gtest/gtest.h"
#include "planning/global_path_planner.hpp"
#include "planning/path_smoothing.hpp"
#include "planning/track.hpp"
#include "rclcpp/rclcpp.hpp"

bool customComparator(const std::pair<double, double> &a, const std::pair<double, double> &b);

std::vector<std::pair<double, double>> orderVectorOfPairs(
    const std::vector<std::pair<double, double>> &vec);

std::string getCurrentDateTimeAsString();

float round_n(float num, int decimal_places);

std::vector<PathPoint> processTriangulations(std::string filename);

void outlierCalculations(std::string filename, int num_outliers);

std::ostream &operator<<(std::ostream &os, const PathPoint &p);

void extractInfo(const std::string_view& filenameView, int& size, int& n_outliers);

using testing::Eq;
namespace fs = std::filesystem;

/**
 * @brief Execution Time Test
 */
TEST(LocalPathPlanner, delauney) {
    std::string directory_path = "../../src/planning/test/maps/";
    int size;
    int n_outliers;
    for (const auto& entry : fs::directory_iterator(directory_path)) {
        if (fs::is_regular_file(entry.path())) {
            std::string filename = entry.path().filename().string();
            if (filename.find("map_") != std::string::npos) {
                extractInfo(filename, size, n_outliers);
                std::string filePath = "src/planning/test/maps/" + filename;
                outlierCalculations(filePath, n_outliers);
                std::vector<PathPoint> path = processTriangulations(filePath);
            }
        }
    }
}
