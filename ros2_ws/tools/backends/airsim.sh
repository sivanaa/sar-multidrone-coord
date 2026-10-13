# AirSim backend for sim.sh and drone_distance.sh - sourced, not run.
# Placeholder: same backend_* functions as gazebo.sh, not implemented yet.
# AirSim's world frame is already NED, so shared (x, y) should map to its
# vehicle X, Y unchanged (relative to PlayerStart) - unlike Gazebo's swap.

BACKEND_NAME=airsim

backend_not_yet() {
  say "SIM=airsim: $1 is not implemented yet (ros2_ws/tools/backends/airsim.sh)." >&2
  return 1
}

backend_world_up() { backend_not_yet backend_world_up; }
backend_px4_env() { backend_not_yet backend_px4_env; }
backend_px4_setup() { backend_not_yet backend_px4_setup; }
backend_marker() { backend_not_yet backend_marker; }
backend_pose() { backend_not_yet backend_pose; }
backend_stop() { :; }  # nothing started, nothing to stop
