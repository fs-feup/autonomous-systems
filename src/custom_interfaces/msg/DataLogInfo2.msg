# DataLogInfo2.msg - Message definition for data log info 2
# Contains pneumatic line pressure readings and master SDC status

std_msgs/Header header

# Boolean flags for pneumatic line pressures and master SDC
bool pneumatic_line_pressure_1
bool pneumatic_line_pressure_2
bool master_sdc_closed
bool pneumatic_line_pressure