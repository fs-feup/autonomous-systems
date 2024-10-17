#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// -------------- MESAGE CODES --------------

/*
 * value of msg[0] for AS State
 */
#define MASTER_AS_STATE_CODE 0x31

/*
 * value of msg[0] for AS Mission
 */
#define MASTER_AS_MISSION_CODE 0x32

/*
 * value of msg[0] for RR RPM Code
 */
#define TEENSY_C1_RR_RPM_CODE 0x11

/*
 * value of msg[0] for RL RPM Code
 */
#define TEENSY_C1_RL_RPM_CODE 0x12

/*
 * value of msg[0] for AS CU alive message
 */
#define ALIVE_MESSAGE 0x41

/**
 * value of msg[0] for mission finished
*/
#define MISSION_FINISHED_CODE 0x42

/**
 * value of msg[0] for emergency detected by AS CU
*/
#define EMERGENCY_CODE 0x43

/**
 * Payload of the single byte message to reset the steering angle sensor
*/
#define SET_ORIGIN_BOSCH_STEERING_ANGLE_RESET 0x05

/**
 * Payload of the single byte message to set the steering angle sensor origin
*/
#define SET_ORIGIN_BOSCH_STEERING_ANGLE_SET 0x03

/**
 * Message code for Bamocar battery voltage
*/
#define BAMOCAR_BATTERY_VOLTAGE_CODE 0xeb

/**
 * Message code for Bamocar motor speed
*/
#define BAMOCAR_MOTOR_SPEED_CODE 0x30

/*
 * Messager code that publish cmds to bamocar
 */
#define TORQUE_COMMAND_BAMO_BYTE 0x90



// -------------- MESAGE IDs --------------

/*
 * ID used for:
 * Current AS Mission
 * Current AS State
 * left wheel rpm
 */
#define MASTER_STATUS 0x300

/*
 * ID used for:
 * Left wheel rpm
 */
#define TEENSY_C1 0x123

/**
 * ID used for
 * Hydraulic line pressure
 */
#define HYDRAULIC_LINE 0x90

/**
 * ID used for:
 * Setting the origin of the steering motor
 * FORMAT: 
 * - 0x5: command mode (5 for setting origin)
 * - 5d: controller id (set in upper computer cubemars application)
*/
#define SET_ORIGIN_CUBEM_ID 0x55d

/**
 * ID used for:
 * Sending steering motor angle position command (in degrees)
 * FORMAT: 
 * - 0x4: command mode (4 for position mode)
 * - 5d: controller id (set in upper computer cubemars application)
*/
#define STEERING_COMMAND_CUBEM_ID 0x45d

/*
 * ID used for:
 * Steering angle from Steering Actuator
 */
#define STEERING_CUBEM_ID 0x295D

/*
 * ID used for:
 * Steering angle from Bosch
 */
#define STEERING_BOSCH_ID 0xa1

/**
 * ID used for:
 * Setting the origin of the steering angle sensor
*/
#define SET_ORIGIN_BOSCH_STEERING_ANGLE_ID 0x725

/**
 * ID used for:
 * Receiving information from the BAMOCAR
*/
#define BAMO_RESPONSE_ID 0x181

/**
 * ID used for AS_CU messages
*/
#define AS_CU_NODE_ID 0x400

/* ID used for:
 * Publish cmds to bamocar
 */
#define BAMO_COMMAND_ID 0x201

/**
 * ID used for accelerations
 * from Bosch IMU
 */
#define IMU_ACC 0x175

/**
 * ID used for accelerations
 * from Bosch IMU
 */
#define IMU_GYRO 0x179

// -------------- SENSORS CONSTANTS --------------

/*
 * Quantization for the acceleration
 * m/s²/digit
 */
#define QUANTIZATION_ACC (0.0019616)

/*
 * Quantization for the gyro
 * rad/s/digit
 */
#define QUANTIZATION_GYRO (0.01)


/**
 * IMU Acceleration Ranges
 */
#define IMU_ACC_MIN_RANGE (-48.9988064)
#define IMU_ACC_MAX_RANGE (48.9988064)

/**
 * IMU Gyroscope Ranges
 */
#define IMU_GYRO_MIN_RANGE (-300.0)
#define IMU_GYRO_MAX_RANGE (300.0)


// -------------- SAFETY CONSTANTS --------------

// TODO: these values can be obtained through CAN communication
/**
 * Checksum for steering angle sensor
*/
#define BOSCH_SA_INITIAL_CRC 0xff
#define BOSCH_SA_CRC_POLYNOMIAL 0x2f

/**
 * Checksum for IMU
 */
#define CRC8_SAE_J1850_INITIAL_CRC 0xFF
#define CRC8_SAE_J1850_POLYNOMIAL 0x1D

/**
 * Limits for the throttle and steering angle
*/
#define THROTTLE_UPPER_LIMIT 1.0 // Input Limits
#define THROTTLE_LOWER_LIMIT -1.0 // Input Limits
#define BAMOCAR_MAX_RPM 6500
#define BAMOCAR_MAX_CURRENT 73
#define BAMOCAR_MAX_VOLTAGE 600
#define BAMOCAR_MAX_SCALE 32767 // Max of the messages from the bamocar
#define MAX_ACCUMULATOR_CHARGING_CURRENT 30 // In Amps, as per documentation

#define STEERING_UPPER_LIMIT 0.392699 // Input Limits
#define STEERING_LOWER_LIMIT -0.392699 // Input Limits
#define STEERING_UPPER_LIMIT_HEX_CHAR 0x11 // Limit for buffer[1] value
#define STEERING_LOWER_LIMIT_HEX_CHAR 0xee // Limit for buffer[1] value


#endif // CONSTANTS_HPP