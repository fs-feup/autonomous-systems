#include "common_lib/competition_logic/mission_logic.hpp"

std::string get_mission_string(int mission) {
  return MISSION_STRING_MAP.find(static_cast<Mission>(mission))->second;
}