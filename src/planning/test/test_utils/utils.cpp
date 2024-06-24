
#include "utils.hpp"

std::ifstream open_read_file(const std::string &filename) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Start openReadFile");
  std::string filePrefix = ament_index_cpp::get_package_share_directory("planning");
  filePrefix = filePrefix.substr(0, filePrefix.find("install"));
  std::string logger_variable = filePrefix + filename;
  std::ifstream file(filePrefix + filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR opening file: %s\n", logger_variable.c_str());
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Successfully opened %s \n",
                 logger_variable.c_str());
  }
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "End openReadFile");
  return file;
}

std::vector<common_lib::structures::Cone> cone_vector_from_file(const std::string &path) {
  std::string x, y, color;
  std::ifstream trackFile = open_read_file(path);
  std::vector<common_lib::structures::Cone> output;
  while (trackFile >> x >> y >> color) {
    float xValue = stof(x);
    float yValue = stof(y);
    common_lib::competition_logic::Color cone_color;
    if (color == "blue_cone") {
      cone_color = common_lib::competition_logic::Color::BLUE;
    } else if (color == "yellow_cone") {
      cone_color = common_lib::competition_logic::Color::YELLOW;
    } else if (color == "orange_cone") {
      cone_color = common_lib::competition_logic::Color::ORANGE;
    } else if (color == "large_orange_cone") {
      cone_color = common_lib::competition_logic::Color::LARGE_ORANGE;
    } else {
      cone_color = common_lib::competition_logic::Color::UNKNOWN;
    }
    common_lib::structures::Cone cone;
    cone.position.x = static_cast<double>(xValue);
    cone.position.y = static_cast<double>(yValue);
    cone.color = cone_color;
    output.push_back(cone);
  }
  trackFile.close();
  return output;
}

std::pair<std::vector<common_lib::structures::Cone>, std::vector<common_lib::structures::Cone>>
track_from_file(const std::string &path) {
  std::string x, y, color;
  std::ifstream trackFile = open_read_file(path);
  std::vector<common_lib::structures::Cone> left_output;
  std::vector<common_lib::structures::Cone> right_output;
  while (trackFile >> x >> y >> color) {
    float xValue = stof(x);
    float yValue = stof(y);
    common_lib::structures::Cone cone;
    cone.position.x = static_cast<double>(xValue);
    cone.position.y = static_cast<double>(yValue);
    if (color == "blue_cone") {
      cone.color = common_lib::competition_logic::Color::BLUE;
      left_output.push_back(cone);
    } else if (color == "yellow_cone") {
      cone.color = common_lib::competition_logic::Color::YELLOW;
      right_output.push_back(cone);
    }
  }
  trackFile.close();
  return std::make_pair(left_output, right_output);
}

std::vector<common_lib::structures::PathPoint> path_from_file(const std::string &path) {
  std::string x, y, v;
  std::ifstream path_file = open_read_file(path);
  std::vector<common_lib::structures::PathPoint> output;
  while (path_file >> x >> y >> v) {
    float x_coordinate = stof(x);
    float y_coordinate = stof(y);
    float velocity = stof(v);
    common_lib::structures::PathPoint path_point;
    path_point.position.x = static_cast<double>(x_coordinate);
    path_point.position.y = static_cast<double>(y_coordinate);
    path_point.ideal_velocity = static_cast<double>(velocity);
    output.push_back(path_point);
  }
  path_file.close();
  return output;
}

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
void extractInfo(const std::string_view &filenameView, int &size, int &n_outliers) {
  std::string filename(filenameView);
  size_t pos1 = filename.find("_");
  size_t pos2 = filename.find("_", pos1 + 1);
  size_t pos3 = filename.find(".", pos2 + 1);
  std::string size_str = filename.substr(pos1 + 1, pos2 - pos1 - 1);
  std::string n_outliers_str = filename.substr(pos2 + 1, pos3 - pos2 - 1);
  std::istringstream(size_str) >> size;
  std::istringstream(n_outliers_str) >> n_outliers;
}

float consecutive_max_distance(const std::vector<common_lib::structures::Cone> &cones) {
  float maxDistance = 0.0;
  for (size_t i = 1; i < cones.size(); i++) {
    float distance = cones[i].position.euclidean_distance(cones[i - 1].position);
    if (distance > maxDistance) {
      maxDistance = distance;
    }
  }
  return maxDistance;
}

/**
 * @brief Defines the way in which two pairs of doubles should be
 * compared when ordering them (lexicographic comparison)
 *
 * @param a One of the pairs of doubles to be compared
 * @param b One of the pairs of doubles to be compared
 * @return true if a<b
 * @return false if a>b
 */
bool custom_comparator(const std::pair<double, double> &a, const std::pair<double, double> &b) {
  if (a.first != b.first) {
    return a.first < b.first;
  }
  return a.second < b.second;
}

/**
 * @brief orders a vector of pairs to make it easier to compare them
 *
 * @param vec vector of pairs to be ordered
 * @return ordered vector of pairs
 */
std::vector<std::pair<double, double>> orderVectorOfPairs(
    const std::vector<std::pair<double, double>> &vec) {
  std::vector<std::pair<double, double>> result = vec;
  std::sort(result.begin(), result.end(), custom_comparator);
  return result;
}

/**
 * @brief Get current date and time as a string.
 * @return Current date and time as a string in "YYYY-MM-DD-HH:MM" format.
 */
std::string get_current_date_time_as_string() {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm;
  localtime_r(&now_time, &now_tm);

  std::stringstream ss;
  ss << std::put_time(&now_tm, "%Y-%m-%d-%H:%M");
  return ss.str();
}

/**
 * @brief rounds float to n decimal places
 *
 * @param num number to be rounded
 * @param decimal_places number of decimal places
 * @return rounded number
 */
float round_n(float num, int decimal_places) {
  num *= pow(10, decimal_places);
  int intermediate = round(num);
  num = intermediate / pow(10, decimal_places);
  return num;
}

std::ostream &operator<<(std::ostream &os, const common_lib::structures::PathPoint &p) {
  return os << '(' << p.position.x << ", " << p.position.y << ')';
}
