# DataLogInfo3.msg - Message definition for data log info 3
# Contains hydraulic line pressure readings

std_msgs/Header header

# Hydraulic pressure readings (32-bit values)
uint32 hydraulic_line_front_pressure
uint32 hydraulic_line_pressure