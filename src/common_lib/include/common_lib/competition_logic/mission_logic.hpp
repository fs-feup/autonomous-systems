#pragma once

#include <map>
#include <string>
#include <unordered_map>

#include "eufs_msgs/msg/can_state.hpp"

namespace common_lib::competition_logic {

enum class Mission {
  ACCELERATION = 0,
  SKIDPAD = 1,
  AUTOCROSS = 2,
  TRACKDRIVE = 3,
  EBS_TEST = 4,
  INSPECTION = 5,
  MANUAL = 6,
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

// the message type is not accepted in map - not recognized by std namespace. Cast to identical type
const std::unordered_map<uint16_t, Mission> eufs_to_system = {
    {static_cast<uint16_t>(eufs_msgs::msg::CanState::AMI_ACCELERATION), Mission::ACCELERATION},
    {static_cast<uint16_t>(eufs_msgs::msg::CanState::AMI_SKIDPAD), Mission::SKIDPAD},
    {static_cast<uint16_t>(eufs_msgs::msg::CanState::AMI_AUTOCROSS), Mission::AUTOCROSS},
    {static_cast<uint16_t>(eufs_msgs::msg::CanState::AMI_TRACK_DRIVE), Mission::TRACKDRIVE}};

const std::unordered_map<std::string, Mission> fsds_to_system = {
    {"acceleration", Mission::ACCELERATION},
    {"skidpad", Mission::SKIDPAD},
    {"autocross", Mission::AUTOCROSS},
    {"trackdrive", Mission::TRACKDRIVE}};
}  // namespace common_lib::competition_logic