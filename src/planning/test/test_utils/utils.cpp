#include "utils.hpp"

std::ifstream open_read_file(const std::string &filename) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Start openReadFile");
  std::string file_prefix = ament_index_cpp::get_package_share_directory("planning");
  file_prefix = file_prefix.substr(0, file_prefix.find("install"));
  std::string logger_variable = file_prefix + filename;
  std::ifstream file(file_prefix + filename);
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
  std::string x;
  std::string y;
  std::string color;
  std::ifstream track_file = open_read_file(path);
  std::vector<common_lib::structures::Cone> output;
  while (track_file >> x >> y >> color) {
    float x_value = stof(x);
    float y_value = stof(y);
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
    cone.position.x = static_cast<double>(x_value);
    cone.position.y = static_cast<double>(y_value);
    cone.color = cone_color;
    output.push_back(cone);
  }
  track_file.close();
  return output;
}

std::pair<std::vector<common_lib::structures::Cone>, std::vector<common_lib::structures::Cone>>
track_from_file(const std::string &path) {
  std::string x;
  std::string y;
  std::string color;
  std::ifstream track_file = open_read_file(path);
  std::vector<common_lib::structures::Cone> left_output;
  std::vector<common_lib::structures::Cone> right_output;
  while (track_file >> x >> y >> color) {
    float x_value = stof(x);
    float y_value = stof(y);
    common_lib::structures::Cone cone;
    cone.position.x = static_cast<double>(x_value);
    cone.position.y = static_cast<double>(y_value);
    if (color == "blue_cone") {
      cone.color = common_lib::competition_logic::Color::BLUE;
      left_output.push_back(cone);
    } else if (color == "yellow_cone") {
      cone.color = common_lib::competition_logic::Color::YELLOW;
      right_output.push_back(cone);
    }
  }
  track_file.close();
  return std::make_pair(left_output, right_output);
}

std::vector<common_lib::structures::PathPoint> path_from_file(const std::string &path) {
  std::string x;
  std::string y;
  std::string v;
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

void extract_info(const std::string_view &filename_view, int &size, int &n_outliers) {
  std::string filename(filename_view);
  size_t pos1 = filename.find("_");
  size_t pos2 = filename.find("_", pos1 + 1);
  size_t pos3 = filename.find(".", pos2 + 1);
  std::string size_str = filename.substr(pos1 + 1, pos2 - pos1 - 1);
  std::string n_outliers_str = filename.substr(pos2 + 1, pos3 - pos2 - 1);
  std::istringstream(size_str) >> size;
  std::istringstream(n_outliers_str) >> n_outliers;
}

float consecutive_max_distance(const std::vector<common_lib::structures::Cone> &cones) {
  float max_distance = 0.0;
  for (size_t i = 1; i < cones.size(); i++) {
    auto distance = static_cast<float>(cones[i].position.euclidean_distance(cones[i - 1].position));
    if (distance > max_distance) {
      max_distance = distance;
    }
  }
  return max_distance;
}

std::vector<std::pair<double, double>> order_vector_of_pairs(
    const std::vector<std::pair<double, double>> &vec) {
  std::vector<std::pair<double, double>> result = vec;
  std::sort(result.begin(), result.end());
  return result;
}

std::string get_current_date_time_as_string() {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm;
  localtime_r(&now_time, &now_tm);

  std::stringstream ss;
  ss << std::put_time(&now_tm, "%Y-%m-%d-%H:%M");
  return ss.str();
}

float round_n(float num, int decimal_places) {
  num *= static_cast<float>(pow(10, decimal_places));
  auto intermediate = static_cast<int>(round(num));
  num = static_cast<float>(static_cast<double>(intermediate) / pow(10, decimal_places));
  return num;
}

std::ostream &operator<<(std::ostream &os, const common_lib::structures::PathPoint &p) {
  return os << '(' << p.position.x << ", " << p.position.y << ')';
}
