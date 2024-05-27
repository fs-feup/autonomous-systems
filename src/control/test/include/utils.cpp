#include "test/include/utils.hpp"

std::vector<custom_interfaces::msg::PathPoint> create_path_msg(std::string track_file) {
  std::string track_file_path = "../../src/control/test/assets/" + track_file + ".csv";
  std::ifstream trackFile(track_file_path);

  std::vector<custom_interfaces::msg::PathPoint> pathpoint_array;

  std::string line;

  std::getline(trackFile, line);  // Skip the first line

  while (std::getline(trackFile, line)) {
    std::istringstream iss(line);
    std::string x, y, v;
    if (std::getline(iss, x, ',') && std::getline(iss, y, ',') && std::getline(iss, v)) {
      // Create a PathPoint and add it to the PathPointArray
      custom_interfaces::msg::PathPoint point;
      point.x = std::stod(x);  // string to double
      point.y = std::stod(y);
      point.v = std::stod(v);
      pathpoint_array.push_back(point);
    }
  }
  trackFile.close();
  return pathpoint_array;
};