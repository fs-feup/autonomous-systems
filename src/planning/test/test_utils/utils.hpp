#pragma once

#include <ros/ros.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <string>
#include <vector>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "gtest/gtest.h"
#include "planning/cone_coloring.hpp"
#include "planning/outliers.hpp"
#include "planning/path_calculation.hpp"
#include "planning/smoothing.hpp"
#include "planning/planning.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/files.hpp"
#include "utils/splines.hpp"



using testing::Eq;
namespace fs = std::filesystem;

std::ifstream open_read_file(const std::string &filename);

std::vector<common_lib::structures::Cone> cone_vector_from_file(const std::string &path);

std::pair<std::vector<common_lib::structures::Cone>, std::vector<common_lib::structures::Cone>>
track_from_file(const std::string &path);

std::vector<common_lib::structures::PathPoint> path_from_file(const std::string &path);

/**
 * Extracts the size and number of outliers from a filename.
 *
 * @param filename The filename containing size and number of outliers
 * information.
 * @param size Output parameter to store the extracted size.
 * @param n_outliers Output parameter to store the extracted number of outliers.
 *
 * The filename should be in the format "map_{size}_{n_outliers}.txt", where:
 *   - "size" is an integer representing the size.
 *   - "n_outliers" is an integer representing the number of outliers.
 *
 * Example:
 *   If filename is "map_100_5.txt", size will be set to 100 and n_outliers
 * to 5.
 */
void extractInfo(const std::string_view &filenameView, int &size, int &n_outliers);

float consecutive_max_distance(const std::vector<common_lib::structures::Cone> &cones);

/**
 * @brief Defines the way in which two pairs of doubles should be
 * compared when ordering them (lexicographic comparison)
 *
 * @param a One of the pairs of doubles to be compared
 * @param b One of the pairs of doubles to be compared
 * @return true if a<b
 * @return false if a>b
 */
bool custom_comparator(const std::pair<double, double> &a, const std::pair<double, double> &b);

/**
 * @brief orders a vector of pairs to make it easier to compare them
 *
 * @param vec vector of pairs to be ordered
 * @return ordered vector of pairs
 */
std::vector<std::pair<double, double>> orderVectorOfPairs(const std::vector<std::pair<double, double>> &vec);
/**
 * @brief Get current date and time as a string.
 * @return Current date and time as a string in "YYYY-MM-DD-HH:MM" format.
 */
std::string get_current_date_time_as_string();

/**
 * @brief rounds float to n decimal places
 *
 * @param num number to be rounded
 * @param decimal_places number of decimal places
 * @return rounded number
 */
float round_n(float num, int decimal_places);

std::ostream &operator<<(std::ostream &os, const common_lib::structures::PathPoint &p);

struct PathPointHash {
  std::size_t operator()(const common_lib::structures::PathPoint &p) const {
    // Adjust the hash function for a bigger tolerance
    std::size_t h1 = std::hash<long>()(static_cast<long>(std::round(p.position.x / 0.5)));
    std::size_t h2 = std::hash<long>()(static_cast<long>(std::round(p.position.y / 0.5)));
    return h1 ^ (h2 << 1);  // Combine the hashes using bitwise shift to reduce collisions
  }
};

struct PathPointEqual {
  bool operator()(const common_lib::structures::PathPoint &p1, const common_lib::structures::PathPoint &p2) const {
    // Use a bigger tolerance for equality comparison
    bool result = std::abs(p1.position.x - p2.position.x) < 0.5 &&
                  std::abs(p1.position.y - p2.position.y) < 0.5;
    return result;
  }
};