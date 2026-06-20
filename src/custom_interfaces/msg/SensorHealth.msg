# Sensor health snapshot from the state estimation Overseer.
# names[i] is the physical sensor name; statuses[i] is its health:
#   0 = LIVE, 1 = FAULTY, 2 = DEAD
std_msgs/Header header
string[] names
uint8[] statuses
