# CBBA consensus broadcast (Choi, Brunet & How, 2009).
# Carries this drone's belief about every task it currently knows about,
# plus its own current bundle (ordered list of committed task_ids).

uint8 drone_id
builtin_interfaces/Time stamp

uint32[] known_task_ids
float32[] winning_bids       # index-aligned with known_task_ids
uint8[] winning_agent_ids    # index-aligned; drone_id of current believed winner
float64[] update_times       # index-aligned; seconds, last time this drone updated its belief

uint32[] bundle              # this drone's own committed tasks, in insertion order
