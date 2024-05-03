#pragma once

#include <map>
#include <string>

enum class Mission {
  ACCELERATION = 0,
  SKIDPAD = 1,
  AUTOCROSS = 2,
  TRACKDRIVE = 3,
  EBS_TEST = 4,
  INSPECTION = 5,
  MANUAL = 6
};

const std::map<Mission, std::string> MISSION_STRING_MAP = {{Mission::ACCELERATION, "acceleration"},
                                                           {Mission::SKIDPAD, "skidpad"},
                                                           {Mission::AUTOCROSS, "autocross"},
                                                           {Mission::TRACKDRIVE, "trackdrive"},
                                                           {Mission::EBS_TEST, "ebs_test"},
                                                           {Mission::INSPECTION, "inspection"},
                                                           {Mission::MANUAL, "manual"}};

std::string get_mission_string(int mission);