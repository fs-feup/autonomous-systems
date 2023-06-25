#include "../../include/utils/files.hpp"

#include <string>
#include <vector>

Track* read_track_file(const std::string& filename) {
  std::string filePrefix = rcpputils::fs::current_path().string();
  std::string filePackage = filePrefix + "/planning/planning/files/" + filename;
  Track* track = new Track();
  track->fillTrack(filePackage);
  return track;
}

void write_path_file(const std::string& filename, std::vector<Position*> path) {
  std::string filePrefix = rcpputils::fs::current_path().string();
  std::string finalPath = filePrefix + "/planning/planning/files/" + filename;
  std::ofstream finalPathFile(finalPath);
  for (size_t i = 0; i < path.size(); i++)
    finalPathFile << path[i]->getX() << " " << path[i]->getY() << "\n";
  finalPathFile.close();
}