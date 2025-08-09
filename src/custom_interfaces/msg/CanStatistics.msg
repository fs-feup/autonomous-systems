std_msgs/Header header
float32 load           # Bus load percentage (0.0 - 100.0)
uint32 tx_error_count  # Optional, transmit error counter
uint32 rx_error_count  # Optional, receive error counter
uint32 overrun_count   # Optional, CAN message overruns count
uint8 status           # Optional status flags (e.g. 0=OK, 1=Warning, 2=Error)
