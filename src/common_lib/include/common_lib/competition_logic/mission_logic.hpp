#pragma once

#include <map>
#include <string>
#include <unordered_map>

#include "eufs_msgs/msg/can_state.hpp"

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

// the message type is not accepted in map - not recognized by std namespace. Cast to identical type
Mission get_mission_from_eufs(unsigned short eufs_mission);
const std::unordered_map<unsigned short, Mission> eufs_to_system = {
    {eufs_msgs::msg::CanState::AMI_ACCELERATION, Mission::ACCELERATION},
    {eufs_msgs::msg::CanState::AMI_SKIDPAD, Mission::SKIDPAD},
    {eufs_msgs::msg::CanState::AMI_AUTOCROSS, Mission::AUTOCROSS},
    {eufs_msgs::msg::CanState::AMI_TRACK_DRIVE, Mission::TRACKDRIVE},
    {eufs_msgs::msg::CanState::AMI_ADS_EBS, Mission::EBS_TEST},
    {eufs_msgs::msg::CanState::AMI_ADS_INSPECTION, Mission::INSPECTION},
    {eufs_msgs::msg::CanState::AMI_NOT_SELECTED, Mission::NONE},
    {eufs_msgs::msg::CanState::AMI_MANUAL, Mission::MANUAL}};

const std::unordered_map<std::string, Mission> fsds_to_system = {
    {"acceleration", Mission::ACCELERATION},
    {"skidpad", Mission::SKIDPAD},
    {"autocross", Mission::AUTOCROSS},
    {"trackdrive", Mission::TRACKDRIVE}};
}  // namespace common_lib::competition_logic