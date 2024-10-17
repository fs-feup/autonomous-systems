#ifndef SAFETY_MECHANISMS_HPP
#define SAFETY_MECHANISMS_HPP

#include <assert.h>
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "utils/constants.hpp"

// DO NOT CHANGE ANYTHING IN THIS FILE

/**
 * @brief Checks if the steering angle is within the safe limits
 * If not, the program will enter in fatal error
 *
 * @param steering_payload_data steering buffer
 */
void check_steering_safe(void *steering_payload_data) {
  // DO NOT REMOVE NEXT BLOCK
  unsigned char hex_value =
      static_cast<unsigned char>((static_cast<char *>(steering_payload_data))[1]);
  unsigned char hex_sign =
      static_cast<unsigned char>((static_cast<char *>(steering_payload_data))[0]);
  // SIGN OF THE ANGLE
  assert(hex_sign == 0x00 || hex_sign == 0xff);
  // HARD STEERING UPPER LIMIT
  assert(hex_sign != 0x00 || hex_value < STEERING_UPPER_LIMIT_HEX_CHAR);
  // HARD STEERING LOWER LIMIT
  assert(hex_sign != 0xff || hex_value > STEERING_LOWER_LIMIT_HEX_CHAR);
  // DO NOT REMOVE PREVIOUS BLOCK
}

/**
 * @brief Checks if the throttle value is within the safe limits
 * If not, the program will enter in fatal error
 *
 * @param throttle_payload_data throttle buffer
 */
void check_throttle_safe(void *throttle_payload_data) {
  // DO NOT REMOVE NEXT BLOCK
  char byte1 = static_cast<unsigned char>((static_cast<char *>(throttle_payload_data))[2]);
  char byte0 = static_cast<unsigned char>((static_cast<char *>(throttle_payload_data))[1]);
  int val = ((byte1 << 8) | (byte0 & 0xff));
  RCLCPP_DEBUG(rclcpp::get_logger("ros_can"), "Val: %d", val);
  // HARD THROTTLE UPPER LIMIT
  assert(val <= BAMOCAR_MAX_SCALE);
  // HARD THROTTLE LOWER LIMIT
  assert(val >= -BAMOCAR_MAX_SCALE);
  // DO NOT REMOVE PREVIOUS BLOCK
}

/**
 * @brief Determines the max torque for braking 
 * given the current rpm of the motor
 * and battery voltage. Follows the formula established in latex
 *
 * @param voltage Voltage value (CAN format)
 * @param rpm Motor speed value (CAN format)
 * @return int Max torque value (CAN format)
 */
int max_torque_dynamic_limits(int voltage, int rpm) {
  rpm = std::max(rpm, 1);
  double voltage_in_v = voltage * BAMOCAR_MAX_VOLTAGE / BAMOCAR_MAX_SCALE;
  double motor_angular_speed = (rpm * BAMOCAR_MAX_RPM / BAMOCAR_MAX_SCALE) * 2 * M_PI / 60;
  double max_torque_in_nm = MAX_ACCUMULATOR_CHARGING_CURRENT * voltage_in_v / motor_angular_speed;
  int result = -(max_torque_in_nm * (sqrt(2) * BAMOCAR_MAX_SCALE) / (0.75 * BAMOCAR_MAX_CURRENT));
  return std::min(result, 0);
}

/**
 * @brief Checks if the CRC8 of the bosch steering angle msg is correct
 *
 * @param msg the CAN msg
 * @return true if the CRC8 is correct
 */
bool checkCRC8(const unsigned char msg[8]) {
  unsigned char received_crc = msg[0];                  // First byte is the CRC
  unsigned char calculated_crc = BOSCH_SA_INITIAL_CRC;  // Initial value
  unsigned char polynomial = BOSCH_SA_CRC_POLYNOMIAL;   // CRC polynomial

  for (int i = 1; i < 8; i++) {
    calculated_crc ^= msg[i];       // XOR byte into CRC
    for (int i = 0; i < 8; i++) {   // Process each bit
      if (calculated_crc & 0x80) {  // High bit set?
        calculated_crc = (calculated_crc << 1) ^ polynomial;
      } else {
        calculated_crc <<= 1;
      }
    }
  }

  calculated_crc ^= 0xFF;  // Final XOR value

  return calculated_crc == received_crc;
}

/**
 * @brief Calculates and verifies the CRC8 checksum for a message based on the SAE J1850 protocol.
 * 
 * @param msg The input message, an array of 8 bytes. The first 7 bytes contain the data, 
 * and the 8th byte contains the received CRC value.
 * @return bool True if the calculated CRC matches the received CRC, indicating the message 
 * integrity is valid. False if there is a mismatch.
 */
bool calculateCRC8_SAE_J1850(const unsigned char msg[8]) {
    unsigned char received_crc = msg[7]; 
    unsigned char calculated_crc = CRC8_SAE_J1850_INITIAL_CRC;
    unsigned char polynomial = CRC8_SAE_J1850_POLYNOMIAL;
    
    for (int i = 0; i < 7; i++) {
        calculated_crc ^= msg[i];
        for (int j = 0; j < 8; ++j) {
            if (calculated_crc & 0x80) {
                calculated_crc = (calculated_crc << 1) ^ polynomial;
            } else {
                calculated_crc <<= 1;
            }
        }
    }
    return calculated_crc == received_crc;
}

#endif  // SAFETY_MECHANISMS_HPP