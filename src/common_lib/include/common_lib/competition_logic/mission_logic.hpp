#pragma once

#include <map>
#include <string>
#include <unordered_map>

namespace common_lib::competition_logic {

enum class Mission {
  MANUAL = 0,
  ACCELERATION = 1,
  SKIDPAD = 2,
  AUTOCROSS = 3,
  TRACKDRIVE = 4,
  EBS_TEST = 5,
  INSPECTION = 6,
  NONE = 7
};

bool operator==(const Mission& mission, const int& value);

bool operator==(const int& value, const Mission& mission);

const std::map<Mission, std::string> MISSION_STRING_MAP = {{Mission::ACCELERATION, "acceleration"},
                                                           {Mission::SKIDPAD, "skidpad"},
                                                           {Mission::AUTOCROSS, "autocross"},
                                                           {Mission::TRACKDRIVE, "trackdrive"},
                                                           {Mission::EBS_TEST, "ebs_test"},
                                                           {Mission::INSPECTION, "inspection"},
                                                           {Mission::MANUAL, "manual"},
                                                           {Mission::NONE, "none"}};

std::string get_mission_string(int mission);

Mission get_mission_from_string(const std::string& name);

}  // namespace common_lib::competition_logic