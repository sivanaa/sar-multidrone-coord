# Published once when a drone's perception pipeline detects a new target
# (e.g. human_detect finding a person). Triggers CBBA task creation fleet-wide.

uint8 drone_id
builtin_interfaces/Time stamp

uint32 target_id
geometry_msgs/Point position
string target_type
float32 confidence
