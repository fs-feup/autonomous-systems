#include "common_lib/competition_logic/mission_logic.hpp"

namespace common_lib::competition_logic {
std::string get_mission_string(int mission) {
  return MISSION_STRING_MAP.find(static_cast<Mission>(mission))->second;
}

bool operator==(const Mission& mission, const int& value) {
  return static_cast<int>(mission) == value;
}

bool operator==(const int& value, const Mission& mission) {
  return static_cast<int>(mission) == value;
}

Mission get_mission_from_eufs(unsigned short eufs_mission) {
  return eufs_to_system.find(eufs_mission)->second;
}

}  // namespace common_lib::competition_logic