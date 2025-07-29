std_msgs/Header header

uint16 error_bitmap_1
uint16 error_bitmap_2
float32 bms_current

bool discharge_limit_enforcement_fault      # P0A07 - Bit 1
bool charger_safety_relay_fault             # P0A08 - Bit 2
bool internal_hardware_fault                # P0A09 - Bit 3
bool internal_heatsink_thermistor_fault     # P0A0A - Bit 4
bool internal_software_fault                # P0A0B - Bit 5
bool highest_cell_voltage_too_high_fault    # P0A0C - Bit 6
bool lowest_cell_voltage_too_low_fault      # P0A0E - Bit 7
bool pack_too_hot_fault                     # P0A10 - Bit 8

bool internal_communication_fault           # P0A1F - Bit 1
bool cell_balancing_stuck_off_fault         # P0A12 - Bit 2
bool weak_cell_fault                        # P0A80 - Bit 3
bool low_cell_voltage_fault                 # P0AFA - Bit 4
bool open_wiring_fault                      # P0A04 - Bit 5
bool current_sensor_fault                   # P0AC0 - Bit 6
bool highest_cell_voltage_over_5v_fault     # P0A0D - Bit 7
bool cell_asic_fault                        # P0A0F - Bit 8
bool weak_pack_fault                        # P0A02 - Bit 9
bool fan_monitor_fault                      # P0A81 - Bit 10
bool thermistor_fault                       # P0A9C - Bit 11
bool external_communication_fault           # U0100 - Bit 12
bool redundant_power_supply_fault           # P0560 - Bit 13
bool high_voltage_isolation_fault           # P0AA6 - Bit 14
bool input_power_supply_fault               # P0A05 - Bit 15
bool charge_limit_enforcement_fault         # P0A06 - Bit 16