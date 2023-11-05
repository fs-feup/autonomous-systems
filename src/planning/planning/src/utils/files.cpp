#include "utils/files.hpp"

#include <filesystem>
#include <string>
#include <vector>

std::vector<Position*> read_path_file(const std::string& filename) {
  std::string filePrefix = ament_index_cpp::get_package_share_directory("planning");
  std::string filepath = filePrefix + "/planning/planning" + filename;
  std::ifstream path_file(filepath);
  std::vector<Position*> path;
  float x, y;
  while (path_file >> x >> y) {
    // std::cout << "x: " << x << " y: " << y << "\n";
    Position* position = new Position(x, y);
    path.push_back(position);
  }
  path_file.close();
  return path;
}

std::ofstream openWriteFile(const std::string& filename) {
  std::string filePrefix = ament_index_cpp::get_package_share_directory("planning");
  filePrefix = filePrefix.substr(0, filePrefix.find("install"));
  std::ofstream file(filePrefix + filename, std::ios::app);
  return file;
}

std::ifstream openReadFile(const std::string& filename) {
  std::string filePrefix = ament_index_cpp::get_package_share_directory("planning");
  filePrefix = filePrefix.substr(0, filePrefix.find("install"));
  std::ifstream file(filePrefix + filename);
  return file;
}
// Track* read_track_file(const std::string& filename) {
//   std::string filePrefix = rcpputils::fs::current_path().string();
//   std::string filePackage = filePrefix + "/planning/planning/files/" + filename;
//   Track* track = new Track();
//   track->fillTrack(filePackage);
//   return track;
// }

