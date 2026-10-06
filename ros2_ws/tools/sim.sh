#!/bin/bash
# One-command two-drone Gazebo runs: everything (Gazebo, both PX4s, both DDS
# agents, coordination nodes, distance monitor) starts in ONE tmux session
# called "sim", one window per process, with the per-launch PX4 setup typed
# into each PX4 shell for you - instead of 7-8 terminals by hand (see
# ARCHITECTURE.md "Launch sequence" for what each step is and why).
#
# Usage, from any WSL terminal:
#   bash sim.sh test-search [--headless]  both drones search, then two targets
#                                    appear mid-search (bid, fly, resume search)
#   bash sim.sh test-bids [--headless]    search, then 4 targets at once in two clusters
#   bash sim.sh test-battery [--headless] drone 1 at 35% bids less, then hits reserve and lands
#   bash sim.sh test-handoff [--headless] drone 1 wins targets, then hits reserve mid-flight:
#                                    hands them to drone 0, flies home, lands
#   bash sim.sh test-b [--headless]  normal spawn, both nodes, two targets
#   bash sim.sh test-c [--headless]  head-on crossing: drone 1 hovers in drone 0's path
#   bash sim.sh up [--headless]      just the sim (Gazebo, PX4 x2, agents), no nodes
#   bash sim.sh nodes                both coordination nodes + distance monitor
#   bash sim.sh detect X Y [N]       target at shared-frame (X,Y), seen by drone N (default 0)
#   bash sim.sh marker X Y [R G B]   visual-only marker at shared-frame (X,Y)
#   bash sim.sh distance             live drone distance, in this terminal
#   bash sim.sh attach               look at the windows (Ctrl+b w = pick a window,
#                                    Ctrl+b d = leave; everything keeps running)
#   bash sim.sh down                 save every window's log to ~/sim_runs/, print the
#                                    closest drone-to-drone distance, stop everything
#   bash sim.sh repeat N [SECONDS]   run test-search N times headless, SECONDS each
#                                    (default 120), and list the closest distances
#
# Paths below are this WSL machine's (see ARCHITECTURE.md) - adjust if yours differ.

REPO=~/sar-multidrone-coord
PX4=~/PX4-Autopilot
AGENT=~/Micro-XRCE-DDS-Agent-243/build/MicroXRCEAgent
SESSION=sim
RUNS=~/sim_runs  # `down` saves each run's logs here, plus a line in summary.txt
SELF="$(realpath "$0")"
ROS_ENV="source /opt/ros/humble/setup.bash && source $REPO/ros2_ws/install/setup.bash"
NODE_ARGS="-p num_drones:=2 -p use_px4_offboard:=true -p hold_altitude:=3.0"
NODE_ARGS+=" -p area_center_x:=4.0 -p area_center_y:=4.0 -p area_radius:=6.0"

say() { echo "[sim] $*"; }

window() {  # window NAME COMMAND - run COMMAND in a new window's own shell,
            # so the window stays open (with any error) if COMMAND exits
  tmux new-window -d -t "$SESSION" -n "$1"
  tmux send-keys -t "$SESSION:$1" "$2" Enter
}

wait_for() {  # wait_for WINDOW TEXT SECONDS - true once TEXT shows up in WINDOW
  local i
  for ((i = 0; i < $3; i++)); do
    tmux capture-pane -p -t "$SESSION:$1" -S -3000 | grep -q "$2" && return 0
    sleep 1
  done
  return 1
}

save_run() {  # every window's full scrollback -> ~/sim_runs/<time>_<test>/
  local name dir w closest
  name=$(tmux show-environment -t "$SESSION" RUN_NAME 2>/dev/null | cut -d= -f2)
  dir="$RUNS/$(date +%F_%H%M%S)_${name:-manual}"
  mkdir -p "$dir"
  # -J re-joins lines tmux wrapped to fit a narrow attached terminal
  for w in $(tmux list-windows -t "$SESSION" -F '#W'); do
    tmux capture-pane -p -J -t "$SESSION:$w" -S - > "$dir/$w.log"
  done
  closest=$(grep -o 'closest [0-9.]* m' "$dir/dist.log" 2>/dev/null | tail -1)
  echo "$(basename "$dir")  ${closest:-no distance recorded}" >> "$RUNS/summary.txt"
  say "logs saved to $dir"
  say "drone-to-drone ${closest:-distance: not recorded (no dist window)}"
}

down() {
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    save_run
    tmux send-keys -t "$SESSION:coord0" C-c 2>/dev/null
    tmux send-keys -t "$SESSION:coord1" C-c 2>/dev/null
    sleep 2
    tmux kill-session -t "$SESSION"
  fi
  pkill -f "gz sim"; pkill -f MicroXRCEAgent; pkill -f "px4_sitl_default/bin/px4"
  pkill -f coordination_node; pkill -f "ros2 topic pub"
  sleep 2
  say "stopped."
}

up() {
  local headless="" n i
  [ "${1:-}" = "--headless" ] && headless="--headless"
  down
  tmux new-session -d -s "$SESSION" -n shell -x 220 -y 50

  say "starting Gazebo (shared world - must be up before any PX4)..."
  window gz "cd $PX4 && export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4/build/px4_sitl_default/src/modules/simulation/gz_plugins && python3 Tools/simulation/gz/simulation-gazebo $headless"
  for ((i = 0; i < 120; i++)); do
    gz topic -l 2>/dev/null | grep -q '^/world/default/clock' && break
    sleep 1
  done
  [ "$i" -ge 120 ] && { say "Gazebo didn't come up in 120s - look at window gz (bash $SELF attach)."; return 1; }

  for n in 0 1; do
    say "starting PX4 instance $n..."
    window "px4_$n" "cd $PX4 && PX4_UXRCE_DDS_PORT=$((8888 + n)) PX4_SYS_AUTOSTART=4001 PX4_SIMULATOR=gz PX4_GZ_MODEL_POSE=$((5 * n)),$((5 * n)) PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i $n"
    wait_for "px4_$n" 'pxh>' 90 || { say "PX4 $n didn't start - look at window px4_$n."; return 1; }
    window "agent$n" "$AGENT udp4 -p $((8888 + n))"
  done

  # Runtime-only PX4 settings, needed after every launch (ARCHITECTURE.md
  # "Required in each PX4 shell after every launch").
  for n in 0 1; do
    tmux send-keys -t "$SESSION:px4_$n" "param set NAV_DLL_ACT 0" Enter
    tmux send-keys -t "$SESSION:px4_$n" "sensor_baro_sim start" Enter
    # PX4 SITL saves every `param set` to its parameters.bson and loads it
    # on the next launch, so the battery tests' settings would leak into
    # later runs (drone 1 booted draining to 20% and hit reserve before
    # any targets existed, 2026-10-06). Back to the defaults every launch.
    tmux send-keys -t "$SESSION:px4_$n" "param set SIM_BAT_DRAIN 60" Enter
    tmux send-keys -t "$SESSION:px4_$n" "param set SIM_BAT_MIN_PCT 50" Enter
  done
  for n in 0 1; do
    wait_for "px4_$n" 'Ready for takeoff' 60 \
      || say "PX4 $n hasn't said 'Ready for takeoff' yet - carrying on (nodes retry ARM)."
  done
  say "sim is up."
}

node() {  # node N INITIAL_X INITIAL_Y
  window "coord$1" "$ROS_ENV && ros2 run coordination_node coordination_node --ros-args -p drone_id:=$1 -p initial_x:=$2 -p initial_y:=$3 $NODE_ARGS"
}

distance_window() {
  window dist "bash $REPO/ros2_ws/tools/drone_distance.sh"
}

wait_airborne() {  # wait_airborne N
  say "waiting for drone $1 to take off..."
  wait_for "px4_$1" 'Takeoff detected' 90 \
    || { say "drone $1 never took off - look at windows px4_$1 / coord$1."; return 1; }
  sleep 4  # finish climbing before handing out targets
}

detect() {  # detect X Y [N]
  say "target at ($1, $2), detected by drone ${3:-0}"
  bash -c "$ROS_ENV && ros2 service call /drone_${3:-0}/coordination/detect_target coordination_msgs/srv/DetectTarget '{position: {x: $1, y: $2, z: 0.0}, target_type: person, confidence: 0.9}'" | tail -1
}

marker() {  # marker X Y [R G B] - shared frame, so Gazebo pose is (Y, X)
  local rgb="${3:-1} ${4:-0} ${5:-0}" name="marker_$(date +%s%N)"
  local sdf="<?xml version='1.0'?><sdf version='1.9'><model name='$name'><static>true</static><pose>$2 $1 0.5 0 0 0</pose><link name='l'><visual name='v'><geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry><material><ambient>$rgb 1</ambient><diffuse>$rgb 1</diffuse></material></visual></link></model></sdf>"
  gz service -s /world/default/create --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 3000 --req "sdf: \"$sdf\"" >/dev/null \
    || say "marker at ($1, $2) failed (cosmetic only - carrying on)"
}

test_b() {
  up "$@" || return 1
  tmux set-environment -t "$SESSION" RUN_NAME test-b
  node 0 0.0 0.0
  node 1 5.0 5.0
  distance_window
  wait_airborne 0 && wait_airborne 1 || return 1
  marker 3 4 1 0 0
  detect 3 4 0
  sleep 10
  marker 6 7 0 1 0
  detect 6 7 1
  say "Test B running: red (3,4) should go to drone 0, green (6,7) to drone 1."
  say "watch: bash $SELF attach  ->  Ctrl+b w  ->  dist"
}

test_search() {
  up "$@" || return 1
  tmux set-environment -t "$SESSION" RUN_NAME test-search
  node 0 0.0 0.0
  node 1 5.0 5.0
  distance_window
  wait_airborne 0 && wait_airborne 1 || return 1
  say "both searching - watch windows coord0/coord1 ('searching: at ..., area explored ...')."
  sleep 15
  marker 2 8 1 0 0
  detect 2 8 0
  say "target 1 at (2, 8) - check coord0/coord1 for the bids and who wins."
  sleep 25
  marker 8 1 0 1 0
  detect 8 1 1
  say "target 2 at (8, 1). After reaching a target, the winner says 'back to searching'."
  say "watch: bash $SELF attach -> Ctrl+b w -> coord0 / coord1 / dist"
}

test_bids() {
  up "$@" || return 1
  tmux set-environment -t "$SESSION" RUN_NAME test-bids
  node 0 0.0 0.0
  node 1 5.0 5.0
  distance_window
  wait_airborne 0 && wait_airborne 1 || return 1
  say "both searching for 15s..."
  sleep 15
  # Four targets at once, two clusters: the right split is one cluster per
  # drone, each drone flying its two in a row (see cbba.py).
  marker 0 8 1 0 0; marker 1 9 1 0 0; marker 8 0 0 1 0; marker 9 1 0 1 0
  detect 0 8 0; detect 1 9 0; detect 8 0 0; detect 9 1 0
  say "4 targets: red (0,8)+(1,9), green (8,0)+(9,1). Expect one cluster per drone;"
  say "coord0/coord1 show each bid, who wins, and 'reached target N - on to the next one'."
  say "watch: bash $SELF attach -> Ctrl+b w -> coord0 / coord1 / dist"
}

test_battery() {
  up "$@" || return 1
  tmux set-environment -t "$SESSION" RUN_NAME test-battery
  # PX4 SITL simulates the battery: SIM_BAT_DRAIN = seconds from full to
  # empty while armed, SIM_BAT_MIN_PCT = where it stops. A fast drain makes
  # drone 1's battery follow MIN_PCT within seconds of each change.
  tmux send-keys -t "$SESSION:px4_1" "param set SIM_BAT_DRAIN 20" Enter
  node 0 0.0 0.0
  node 1 5.0 5.0
  distance_window
  wait_airborne 0 && wait_airborne 1 || return 1
  say "both searching for 15s (batteries at 50% = full-strength bids)..."
  sleep 15
  say "drone 1 battery -> 35%: its bids should drop to ~0.4x."
  tmux send-keys -t "$SESSION:px4_1" "param set SIM_BAT_MIN_PCT 35" Enter
  sleep 6
  marker 0 8 1 0 0; marker 1 9 1 0 0; marker 8 0 0 1 0; marker 9 1 0 1 0
  detect 0 8 0; detect 1 9 0; detect 8 0 0; detect 9 1 0
  sleep 4
  say "drone 1 battery -> 20% (reserve is 25%): it should hand off its targets,"
  say "fly home to (5, 5) and land; drone 0 should end up with all 4."
  tmux send-keys -t "$SESSION:px4_1" "param set SIM_BAT_MIN_PCT 20" Enter
  say "watch: bash $SELF attach -> Ctrl+b w -> coord0 / coord1 / dist"
}

test_handoff() {
  up "$@" || return 1
  tmux set-environment -t "$SESSION" RUN_NAME test-handoff
  node 0 0.0 0.0
  node 1 5.0 5.0
  distance_window
  wait_airborne 0 && wait_airborne 1 || return 1
  say "both searching for 15s..."
  sleep 15
  # Both healthy, so drone 1 wins its share (one cluster, as in test-bids)...
  marker 0 8 1 0 0; marker 1 9 1 0 0; marker 8 0 0 1 0; marker 9 1 0 1 0
  detect 0 8 0; detect 1 9 0; detect 8 0 0; detect 9 1 0
  # ...then runs low on the way to them - right away and near-instantly
  # (1s full-to-empty): with a 3s wait and a 10s drain, drone 1 finished
  # its nearby pair before reaching reserve, so there was nothing to hand
  # off (2026-10-06).
  say "drone 1 battery -> 20% mid-flight: it should hand its targets to drone 0,"
  say "fly home to (5, 5) and land; drone 0 should end up doing all 4."
  tmux send-keys -t "$SESSION:px4_1" "param set SIM_BAT_DRAIN 1" Enter
  tmux send-keys -t "$SESSION:px4_1" "param set SIM_BAT_MIN_PCT 20" Enter
  say "watch: bash $SELF attach -> Ctrl+b w -> coord0 / coord1 / dist"
}

repeat_search() {  # repeat_search N SECONDS - for measuring separation
  local n=${1:-3} secs=${2:-120} i before
  mkdir -p "$RUNS" && touch "$RUNS/summary.txt"
  before=$(wc -l < "$RUNS/summary.txt")
  for ((i = 1; i <= n; i++)); do
    say "=== run $i of $n ==="
    test_search --headless || say "run $i failed to start - its logs are saved too."
    sleep "$secs"
    down
  done
  say "closest drone-to-drone distance per run:"
  tail -n +"$((before + 1))" "$RUNS/summary.txt"
}

test_c() {
  local gx gy i
  up "$@" || return 1
  tmux set-environment -t "$SESSION" RUN_NAME test-c

  # Drone 1 hovers at its spawn under PX4's own control; its coordination
  # node does NOT run (it would search and bid instead of holding still).
  say "drone 1: taking off to hover at its spawn..."
  tmux send-keys -t "$SESSION:px4_1" "param set MIS_TAKEOFF_ALT 3" Enter
  for i in 1 2 3 4 5; do
    tmux send-keys -t "$SESSION:px4_1" "commander takeoff" Enter
    wait_for px4_1 'Takeoff detected' 8 && break
  done
  wait_for px4_1 'Takeoff detected' 1 \
    || { say "drone 1 won't take off - look at window px4_1."; return 1; }
  sleep 8

  # Tell drone 0 where drone 1 really is (Gazebo (x,y) = shared (y,x)).
  read -r gx gy _ <<< "$(gz model -m x500_1 -p 2>/dev/null | grep -A1 XYZ | tail -1 | tr -d '[]')"
  [ -z "$gx" ] && { say "couldn't read drone 1's position from Gazebo."; return 1; }
  say "drone 1 hovering at shared ($gy, $gx)"
  window drone1_pos "$ROS_ENV && ros2 topic pub -r 2 /drone_1/coordination/agent_state coordination_msgs/msg/AgentState \"{drone_id: 1, position: {x: $gy, y: $gx, z: 0.0}, best_position: {x: $gy, y: $gx, z: 0.0}, best_fitness: 0.0, state: 0}\""

  node 0 0.0 0.0
  distance_window
  wait_airborne 0 || return 1
  marker 10 10 0 1 0
  detect 10 10 0
  say "Test C running: drone 0 should curve around drone 1 and reach (10, 10);"
  say "closest distance should stay >= 1.5m. watch: bash $SELF attach -> Ctrl+b w -> dist"
}

case "${1:-}" in
  up) shift; up "$@" ;;
  nodes) node 0 0.0 0.0; node 1 5.0 5.0; distance_window; say "nodes started." ;;
  test-search) shift; test_search "$@" ;;
  test-bids) shift; test_bids "$@" ;;
  test-battery) shift; test_battery "$@" ;;
  test-handoff) shift; test_handoff "$@" ;;
  repeat) shift; repeat_search "$@" ;;
  test-b) shift; test_b "$@" ;;
  test-c) shift; test_c "$@" ;;
  detect) shift; detect "$@" ;;
  marker) shift; marker "$@" ;;
  distance) bash "$REPO/ros2_ws/tools/drone_distance.sh" ;;
  attach) tmux attach -t "$SESSION" ;;
  down) down ;;
  *) sed -n '2,/^# Paths below/p' "$SELF" | sed 's/^# \{0,1\}//' ;;
esac
