#include <string>
#include <unordered_map>

#include "custom_interfaces/msg/vcu.hpp"
#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @enum Mission
 * @brief Enumeration representing the different missions.
 */
enum class Mission { not_selected, acceleration, skidpad, trackdrive, autocross };

// the message type is not accepted in map - not recognized by std namespace. Cast to identical type
const std::unordered_map<uint16_t, Mission> eufsToSystem = {
    {eufs_msgs::msg::CanState::AMI_ACCELERATION, Mission::acceleration},
    {eufs_msgs::msg::CanState::AMI_SKIDPAD, Mission::skidpad},
    {eufs_msgs::msg::CanState::AMI_AUTOCROSS, Mission::autocross},
    {eufs_msgs::msg::CanState::AMI_TRACK_DRIVE, Mission::trackdrive},
    {eufs_msgs::msg::CanState::AMI_AUTONOMOUS_DEMO, Mission::not_selected},
    {eufs_msgs::msg::CanState::AMI_ADS_INSPECTION, Mission::not_selected},
    {eufs_msgs::msg::CanState::AMI_ADS_EBS, Mission::not_selected},
    {eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_A, Mission::not_selected},
    {eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_B, Mission::not_selected},
    {eufs_msgs::msg::CanState::AMI_JOYSTICK, Mission::not_selected},
    {eufs_msgs::msg::CanState::AMI_MANUAL, Mission::not_selected}};

const std::unordered_map<std::string, Mission> fsdsToSystem = {
    {"acceleration", Mission::acceleration},
    {"skidpad", Mission::skidpad},
    {"autocross", Mission::autocross},
    {"trackdrive", Mission::trackdrive}};

const std::unordered_map<int, Mission> adsdvToSystem = {{1, Mission::acceleration},
                                                        {2, Mission::skidpad},
                                                        {3, Mission::autocross},
                                                        {4, Mission::trackdrive}};
