# DataLogInfo1.msg - Message definition for data log info 1
# Contains various system status flags, state information, and DC voltage readings

std_msgs/Header header

# Boolean flags from first byte (msg[0])
bool pneumatic_line_pressure
bool asms_on
bool asats_pressed
bool ats_pressed
bool tsms_sdc_closed
bool master_sdc_closed
bool ts_on
bool wd_ready

# Boolean flags from second byte (msg[1])
bool emergency_signal
bool bms_dead
bool engage_ebs_check
bool release_ebs_check
bool steer_dead
bool pc_dead
bool inversor_dead
bool res_dead

# State checkup (4 bits from msg[2])
uint8 state_checkup

# DC voltage (32-bit value from msg[3-6])
uint32 dc_voltage

# Mission and state (4 bits each from msg[7])
uint8 mission
uint8 state

# Additional pneumatic line pressure readings
bool pneumatic_line_pressure_1
bool pneumatic_line_pressure_2
bool pneumatic_line_pressure_main

# Placeholder field
bool placeholder