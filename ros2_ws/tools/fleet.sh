# Spawn points, sourced by sim.sh and drone_distance.sh: drone N at
# (SPAWN_X[N], SPAWN_Y[N]) in the fleet's shared frame (drone 0's PX4 local
# NED frame). Each simulator backend converts them to its own world frame;
# the coordination nodes get them unchanged as initial_x/initial_y.
SPAWN_X=(0.0 5.0)
SPAWN_Y=(0.0 5.0)
