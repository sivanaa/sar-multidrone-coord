#!/bin/bash
# One-shot, reproducible demo run: launches two drones, watches them explore
# via PSO for a fixed window, fires a simulated target detection, lets the
# CBBA reaction play out, then cleanly stops everything and leaves a plot.
#
# Run from an already-activated environment (pixi shell in ros_ws, unset
# VIRTUAL_ENV, source ros2_ws/install/setup.bash), from anywhere:
#
#   bash ros2_ws/tools/demo_run.sh [output.png] [search_seconds] [react_seconds] [target_x] [target_y]
#
# Defaults: ~/sivana/sar-multidrone-coord/demo_run.png, 15s search, 10s react,
# target at (3, 4) - roughly midway between drone 0's (0,0) and drone 1's
# (5,5) starts, which is why it was easy to assume one of them just always
# wins. Pass target_x/target_y explicitly to place the target deliberately
# closer to one drone or the other and confirm the winner actually flips
# with distance, e.g.:
#   bash tools/demo_run.sh /tmp/out.png 15 10 1.0 1.0   # near drone 0 - it should win
#   bash tools/demo_run.sh /tmp/out.png 15 10 4.5 4.5   # near drone 1 - it should win

set -e

OUT="${1:-$HOME/sivana/sar-multidrone-coord/demo_run.png}"
SEARCH_SECONDS="${2:-15}"
REACT_SECONDS="${3:-10}"
TARGET_X="${4:-3.0}"
TARGET_Y="${5:-4.0}"
GIF_OUT="${OUT%.png}.gif"
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
  if [ -n "$VIZ_PID" ]; then
    kill "$VIZ_PID" 2>/dev/null
    # The visualizer's SIGTERM handler renders and saves a GIF of the whole
    # run before it actually exits (see visualize_run.py) - without this
    # wait, the script (and this trap) would return to the shell prompt
    # while that render is still in progress, and an immediate scp would
    # grab a truncated/missing file.
    wait "$VIZ_PID" 2>/dev/null
  fi
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
  --gif-out "$GIF_OUT" \
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

echo "== Drone 0 detects a target at (${TARGET_X},${TARGET_Y}) via the real detect_target service =="
# Calls report_target_detected() inside drone 0's own process, so drone 0
# self-bids immediately (same path real perception code would use) AND
# broadcasts TargetDetected on its topic, which drone 1 (and every other
# drone in the fleet) receives and bids on too. This replaced an earlier
# workaround that published the same task directly onto BOTH drones' own
# topics - that faked two independent "detections" instead of exercising
# one real detection with every drone (including the detector) bidding on
# it, since a node never receives its own published topic messages and so
# never self-bid when driven purely by external `ros2 topic pub`.
ros2 service call /drone_0/coordination/detect_target \
  coordination_msgs/srv/DetectTarget \
  "{position: {x: ${TARGET_X}, y: ${TARGET_Y}, z: 0.0}, target_type: 'person', confidence: 0.9}"

echo "== Letting the CBBA reaction play out for ${REACT_SECONDS}s =="
sleep "$REACT_SECONDS"

echo "== Stopping and rendering GIF (a few seconds) =="
echo "== Done. Demo plot: $OUT  |  Animation: $GIF_OUT =="
# cleanup() runs automatically here via the EXIT trap, and now blocks until
# the visualizer has actually finished writing $GIF_OUT (see cleanup()).
