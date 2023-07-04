# ID of the node
uint16 id

# Name of the node
string name

# Next expected heartbeat value
uint8 exp_heartbeat

# Time of last heartbeat
builtin_interfaces/Time last_heartbeat

# Describes the action to take if node is offline
uint8 severity

# Whether the node is running or not according to the status monitor
bool online