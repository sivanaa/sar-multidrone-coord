# Fleet size and spawn points, sourced by sim.sh and drone_distance.sh.
# NUM_DRONES=N bash sim.sh ... (default 2). Drone N spawns at
# (SPAWN_X[N], SPAWN_Y[N]) in the fleet's shared frame (drone 0's PX4 local
# NED frame); each simulator backend converts them to its own world frame,
# and the coordination nodes get them unchanged as initial_x/initial_y.
# Same layout as tools/fleet_sim.py's spawn_points: 2 drones at (0,0) and
# (5,5) as always; 3-4 on a 5m grid, (0,0), (0,5), (5,0), (5,5).
NUM_DRONES=${NUM_DRONES:-2}
SPAWN_X=()
SPAWN_Y=()
if [ "$NUM_DRONES" -eq 2 ]; then
  SPAWN_X=(0.0 5.0)
  SPAWN_Y=(0.0 5.0)
else
  _side=1
  while [ $((_side * _side)) -lt "$NUM_DRONES" ]; do _side=$((_side + 1)); done
  for ((_k = 0; _k < NUM_DRONES; _k++)); do
    SPAWN_X+=("$((_k / _side * 5)).0")
    SPAWN_Y+=("$((_k % _side * 5)).0")
  done
fi
DRONES="$(seq 0 $((NUM_DRONES - 1)) | tr '\n' ' ')"  # "0 1 2 ..."
