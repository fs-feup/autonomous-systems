std_msgs/Header header


# 0 -> Okay, 1 -> Warning, 2 -> Error
int8 bad_params # Reading from EPROM defective
int8 hardware_error # Critical hardware error 
int8 rfe_not_present # Safety circuit not present
int8 bus_timeout # CAN Bus timeout  
int8 bad_missing_feedback # Bad or missing feedback signal
int8 voltage_low  # Volage missing or below threshold
int8 motor_temp # Motor temp max limit exceeded
int8 igbt_temp # Output state temp too high
int8 voltage_high # Voltage too high
int8 i_peak # Overcurrent or strong oscilating current
int8 race_away # Spinning without setpoint
int8 ecode_error # Bad or missing Ecode protocol
int8 watchdog_reset # CPU Reset due to Watchdog reset
int8 i_offset # AC Current Offset detection fault
int8 internal_voltage # Error beacuse of internal Voltage problem
int8 bleed_resistor_overload