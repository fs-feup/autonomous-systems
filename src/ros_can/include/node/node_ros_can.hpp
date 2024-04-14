#include <canlib.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/imu.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

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

/*
 * ID used for imu values
 */
#define MASTER_IMU 0x501

/*
 * ID used for:
 * Steering angle
 */
#define STEERING_ID 0x700

/*
 * ID used from the IMU for:
 * yaw rate
 * acceleration in y
 */
#define IMU_YAW_RATE_ACC_Y_ID 0x174



/*
* ID used from the IMU for: 
* roll rate
* acceleration in x
*/
#define IMU_ROLL_RATE_ACC_X_ID 0x178

/*
* ID used from the IMU for:
* pitch rate
* acceleration in z
*/
#define IMU_PITCH_RATE_ACC_Z_ID 0x17C

class RosCan : public rclcpp::Node {
 private:
  rclcpp::Publisher<custom_interfaces::msg::OperationalStatus>::SharedPtr operationalStatus;
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr wheelRPM;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
  rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr steeringAngle;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr busStatus;
  rclcpp::TimerBase::SharedPtr timer;

  // Holds a handle to the CAN channel
  canHandle hnd;
  // Status returned by the Canlib calls
  canStatus stat;

  /*
   *Counter to keep track of the number of wheel speed msg received during the syncing process
   *goes from 0 to 3, 4 when the sync is complete
   */
  int statusMsgCnt = 0;

  // Current time, to calculate the sync intervals
  rclcpp::Time curTime;

  /*
   *Interval between the 1st and 2nd wheel speed msg received during the syncing process
   */
  uint64_t syncInterval1 = 0.0f;

  /*
   *Interval between the 2nd and 3rd wheel speed msg received during the syncing process
   */
  uint64_t syncInterval2 = 0.0f;

  /*
  * 1 if the first msg received is from C1
  * 0 if the first msg received is from C2
  * Used to determine the sync mode
  */
  bool firstMsgIsC1 = 0;

  /*
   * if syncMode = 0, group rrRPM with rlRPM
   * if syncMode = 1, group rlRPM with rrRPM
   * (only publish when both have been recevied)
   */
  bool syncMode = 0;

  /*
   * rlRPM = left wheel rpm
   * rrRPM = right wheel rpm
   */
  float rlRPM;
  float rrRPM;

  /*
    rrRPMStatus = 0 if rrRPM has not been received/updated
    rrRPMStatus = 1 if rrRPM has been received(ready to be published)
  */
  bool rrRPMStatus = 0;

  /*
    rlRPMStatus = 0 if rlRPM has not been received/updated
    rlRPMStatus = 1 if rlRPM has been received(ready to be published)
  */
  bool rlRPMStatus = 0;

  /*
  goSignal:
  0 - Stop
  1 - Go
  */
  bool goSignal = 0;

  /*Current Mission:
  0 - Manual
  1 - Acceleration
  2 - Skidpad
  3 - Autocross
  4 - Trackdrive
  5 - EBS_Test
  6 - Inspection*/
  int asMission;

  /**
   * @brief Function define the syncMode and calculate sync time
   */
  void syncRPM();

  /**
   * @brief Function to update the wheel rpm
   * @return the wheel rpm
   * @param msg - the CAN msg
   */
  float updatewheelRPM(unsigned char msg[8]);

  /**
   * @brief Function to turn ON and OFF the CAN BUS
   */
  void busStatus_callback(std_msgs::msg::String busStatus);

  /**
   * @brief Function cyclically reads all CAN msg from buffer
   */
  void canSniffer();

  /**
   * @brief Function to interpret the CAN msg
   * @param id - the CAN msg id
   * @param msg - the CAN msg
   * @param dlc - the CAN msg length
   * @param flag - the CAN msg flag - see kvaser documentation for more info
   * @param time - the CAN msg time stamp
   */
  void canInterpreter(long id, unsigned char msg[8], unsigned int dlc, unsigned int flag,
                      unsigned long time);

  /**
   * @brief Function to publish the Operational Status
   */
  void opStatusPublisher();

  /**
   * @brief Function to publish the left wheel rpm
   */
  void WheelRPMPublisher();

  /**
   * @brief Function to publish the imu values
   * @param msg - the CAN msg
   */
  void imuPublisher(unsigned char msg[8]);

  /**
   * @brief Function to publish the steering angle
   * @param angleLSB - the steering angle least significant byte
   * @param angleMSB - the steering angle most significant byte
   */
  void steeringAnglePublisher(unsigned char angleLSB, unsigned char angleMSB);

  /**
   * @brief Function to interpret the master status CAN msg
   * @param msg - the CAN msg
   */
  void canInterpreterMasterStatus(unsigned char msg[8]);

 public:
  /**
   * @brief Contructor for the RosCan class
   */
  RosCan();
};
