#include "communicators/ads-dv.hpp"

#include <stdio.h>
#include <unistd.h>

#include <cstdio>

#include "custom_interfaces/msg/vehicle_command.hpp"
#include "custom_interfaces/msg/vehicle_info.hpp"
#include "fs-ai_api/fs-ai_api.h"
#include "rclcpp/rclcpp.hpp"

AdsDvCommunicator::AdsDvCommunicator(Orchestrator* orchestrator)
    : Communicator(), orchestrator_(orchestrator) {
  if (fs_ai_api_init((char*)"can0", 1, 1)) {
    printf("fs_ai_api_init() failed\r\n");
  }
}

void AdsDvCommunicator::send_to_car(const custom_interfaces::msg::VehicleCommand msg) {
  // TODO: adapt to new structures
  fs_ai_api_vcu2ai vcu2ai_data;
  fs_ai_api_ai2vcu ai2vcu_data;

  // Send handshake bit
  if (HANDSHAKE_RECEIVE_BIT_OFF == vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT) {
    ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF;
  } else if (HANDSHAKE_RECEIVE_BIT_ON == vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT) {
    ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON;
  } else {
    printf("HANDSHAKE_BIT error\r\n");
  }

  // Send mission status
  if (msg.status == 3) {
    ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_FINISHED;
  } else if (msg.status == 2) {
    ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_RUNNING;
  } else if (msg.status == 1) {
    ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_SELECTED;
  } else {
    ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_NOT_SELECTED;
  }

  // Send direction request
  if (msg.direction_request == 1) {
    ai2vcu_data.AI2VCU_DIRECTION_REQUEST = DIRECTION_FORWARD;
  } else {
    ai2vcu_data.AI2VCU_DIRECTION_REQUEST = DIRECTION_NEUTRAL;
  }

  // Send estop request
  if (msg.estop_request == 0) {
    ai2vcu_data.AI2VCU_ESTOP_REQUEST = ESTOP_NO;
  } else {
    ai2vcu_data.AI2VCU_ESTOP_REQUEST = ESTOP_YES;
  }

  ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg = msg.steering_angle_request;
  ai2vcu_data.AI2VCU_AXLE_SPEED_REQUEST_rpm = msg.axle_speed_request;
  ai2vcu_data.AI2VCU_AXLE_TORQUE_REQUEST_Nm = msg.axle_torque_request;
  ai2vcu_data.AI2VCU_BRAKE_PRESS_REQUEST_pct = msg.brake_press_request;

  fs_ai_api_ai2vcu_set_data(&ai2vcu_data);
}

void AdsDvCommunicator::send_from_car() {
  fs_ai_api_vcu2ai vcu2ai_data;
  fs_ai_api_vcu2ai_get_data(&vcu2ai_data);

  custom_interfaces::msg::VehicleInfo msg;

  msg.handshake_receive_bit = vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT;
  msg.res_go_signal = vcu2ai_data.VCU2AI_RES_GO_SIGNAL;
  msg.as_state = vcu2ai_data.VCU2AI_AS_STATE;
  msg.ami_state = vcu2ai_data.VCU2AI_AMI_STATE;
  msg.steering_angle = vcu2ai_data.VCU2AI_STEER_ANGLE_deg;
  msg.brake_press_f_pct = vcu2ai_data.VCU2AI_BRAKE_PRESS_F_pct;
  msg.brake_press_r_pct = vcu2ai_data.VCU2AI_BRAKE_PRESS_R_pct;
  msg.fl_wheel_speed = vcu2ai_data.VCU2AI_FL_WHEEL_SPEED_rpm;
  msg.fr_wheel_speed = vcu2ai_data.VCU2AI_FR_WHEEL_SPEED_rpm;
  msg.rl_wheel_speed = vcu2ai_data.VCU2AI_RL_WHEEL_SPEED_rpm;
  msg.rr_wheel_speed = vcu2ai_data.VCU2AI_RR_WHEEL_SPEED_rpm;
  msg.fl_pulse_count = vcu2ai_data.VCU2AI_FL_PULSE_COUNT;
  msg.fr_pulse_count = vcu2ai_data.VCU2AI_FR_PULSE_COUNT;
  msg.rl_pulse_count = vcu2ai_data.VCU2AI_RL_PULSE_COUNT;
  msg.rr_pulse_count = vcu2ai_data.VCU2AI_RR_PULSE_COUNT;

  this->orchestrator_->publish_info(msg);
}
