# Broadcast periodically by each drone for swarm awareness.
# Used by neighbors both for general status and as the PSO neighbor-best signal.

uint8 STATE_SEARCH=0
uint8 STATE_TASK_ALLOCATION=1

uint8 drone_id
builtin_interfaces/Time stamp

geometry_msgs/Point position        # local frame (meters), matches PX4 local position
geometry_msgs/Point best_position   # this drone's PSO personal-best position so far
float32 best_fitness                # fitness achieved at best_position (higher is better)

uint8 state                         # one of STATE_SEARCH / STATE_TASK_ALLOCATION
