#!/bin/bash
# One-shot, reproducible demo run: launches two drones, watches them explore
# via PSO for a fixed window, fires a simulated target detection, lets the
# CBBA reaction play out, then cleanly stops everything and leaves a plot.
#
# Run from an already-activated environment (pixi shell in ros_ws, unset
# VIRTUAL_ENV, source ros2_ws/install/setup.bash), from anywhere:
#
#   bash ros2_ws/tools/demo_run.sh [output.png] [search_seconds] [react_seconds]
#
# Defaults: ~/sivana/sar-multidrone-coord/demo_run.png, 15s search, 10s react.

set -e

OUT="${1:-$HOME/sivana/sar-multidrone-coord/demo_run.png}"
SEARCH_SECONDS="${2:-15}"
REACT_SECONDS="${3:-10}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DRONE0_PID=""
DRONE1_PID=""
VIZ_PID=""

# Runs no matter how the script exits (success, error, or Ctrl-C) - without
# this, a transient failure partway through (e.g. `ros2 topic pub` hiccups)
# would abort the script via `set -e` and leave the background drone/
# visualizer processes orphaned with nothing to stop them. Uses the same
# pattern-based kill as the startup cleanup below (not just the captured
# PIDs) as a second line of defense - if this SSH session itself drops
# mid-run (as happened repeatedly during real use), this trap never even
# gets to fire, which is exactly why the *next* invocation's startup
# cleanup matters just as much as this one.
cleanup() {
  [ -n "$DRONE0_PID" ] && kill "$DRONE0_PID" 2>/dev/null
  [ -n "$DRONE1_PID" ] && kill "$DRONE1_PID" 2>/dev/null
  [ -n "$VIZ_PID" ] && kill "$VIZ_PID" 2>/dev/null
  pkill -f "lib/coordination_node/coordination_node" 2>/dev/null
  pkill -f "tools/visualize_run.py" 2>/dev/null
  return 0
}
trap cleanup EXIT

echo "== Cleaning up any leftover coordination_node / visualizer processes =="
# NOTE: "coordination_node coordination_node" (space-separated) never
# matched anything real - `ros2 run` execs into the actual binary, whose
# argv shows the *path* "lib/coordination_node/coordination_node" (slash-
# separated), confirmed live via `pgrep -af coordination_node`. The old
# pattern silently did nothing every single run, letting orphaned
# processes from interrupted sessions pile up indefinitely.
pkill -f "lib/coordination_node/coordination_node" 2>/dev/null || true
pkill -f "tools/visualize_run.py" 2>/dev/null || true
sleep 1

echo "== Starting visualizer -> $OUT =="
# Started BEFORE the drones (not after) so its subscriptions are already
# established when they start publishing - otherwise the very first
# samples it sees aren't the true initial_x/initial_y launch position,
# they're wherever each drone has already drifted to during the gap.
python3 "$SCRIPT_DIR/visualize_run.py" --num-drones 2 --out "$OUT" \
  --save-every-sec 2 > /tmp/demo_visualizer.log 2>&1 &
VIZ_PID=$!

sleep 3

echo "== Launching drone 0 (start: 0,0) =="
ros2 run coordination_node coordination_node --ros-args \
  -p drone_id:=0 -p num_drones:=2 -p initial_x:=0.0 -p initial_y:=0.0 \
  > /tmp/demo_drone0.log 2>&1 &
DRONE0_PID=$!

echo "== Launching drone 1 (start: 5,5) =="
ros2 run coordination_node coordination_node --ros-args \
  -p drone_id:=1 -p num_drones:=2 -p initial_x:=5.0 -p initial_y:=5.0 \
  > /tmp/demo_drone1.log 2>&1 &
DRONE1_PID=$!

echo "== Letting PSO search run for ${SEARCH_SECONDS}s =="
sleep "$SEARCH_SECONDS"

echo "== Publishing simulated target detection (drone 1 -> target at 3,4) =="
ros2 topic pub --once /drone_1/coordination/target_detected \
  coordination_msgs/msg/TargetDetected \
  "{drone_id: 1, target_id: 99, position: {x: 3.0, y: 4.0, z: 0.0}, target_type: 'person', confidence: 0.9}"

echo "== Letting the CBBA reaction play out for ${REACT_SECONDS}s =="
sleep "$REACT_SECONDS"

echo "== Done. Demo plot: $OUT =="
# cleanup() runs automatically here via the EXIT trap.
