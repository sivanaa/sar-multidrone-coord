#!/bin/bash
# One-command two-drone simulator runs: everything (the simulator, both PX4s,
# both DDS agents, coordination nodes, distance monitor) starts in ONE tmux
# session called "sim", one window per process, with the per-launch PX4 setup
# typed into each PX4 shell for you - instead of 7-8 terminals by hand (see
# ARCHITECTURE.md "Launch sequence" for what each step is and why).
#
# Simulator: Gazebo by default; SIM=airsim bash sim.sh ... for AirSim (not
# implemented yet). Everything simulator-specific is in backends/$SIM.sh;
# the tests and the coordination nodes are the same for both.
#
# Usage, from any WSL terminal:
#   bash sim.sh demo [--headless]         recorded 3-act run (search, auction, battery
#                                    hand-off); then `down`, then `render` for a GIF
#   bash sim.sh demo-flood [--headless]   recorded search weighted by a synthetic river
#                                    flood-risk map (tools/make_risk_map.py)
#   bash sim.sh render [RUN_DIR]     GIF of the newest recorded run, copied to Windows Videos
#   bash sim.sh test-search [--headless]  both drones search, then two targets
#                                    appear mid-search (bid, fly, resume search)
#   bash sim.sh test-bids [--headless]    search, then 4 targets at once in two clusters
#   bash sim.sh test-battery [--headless] drone 1 at 35% bids less, then hits reserve and lands
#   bash sim.sh test-handoff [--headless] drone 1 wins targets, then hits reserve mid-flight:
#                                    hands them to drone 0, flies home, lands
#   bash sim.sh test-b [--headless]  normal spawn, both nodes, two targets
#   bash sim.sh test-c [--headless]  head-on crossing: drone 1 hovers in drone 0's path
#   bash sim.sh test-x [--headless]  both drones fly through each other's paths head-on, twice
#   bash sim.sh up [--headless]      just the sim (Gazebo, PX4 x2, agents), no nodes
#   bash sim.sh nodes                both coordination nodes + distance monitor
#   bash sim.sh detect X Y [N] [TYPE] target at shared-frame (X,Y), seen by drone N (default 0)
#   bash sim.sh marker X Y [R G B]   visual-only marker at shared-frame (X,Y)
#   bash sim.sh distance             live drone distance, in this terminal
#   bash sim.sh attach               look at the windows (Ctrl+b w = pick a window,
#                                    Ctrl+b d = leave; everything keeps running)
#   bash sim.sh down                 save every window's log to ~/sim_runs/, print the
#                                    closest drone-to-drone distance, stop everything
#   bash sim.sh repeat N [SECONDS] [TEST]  run TEST (default test-search) N times headless, SECONDS each
#                                    (default 120), and list the closest distances
#
# Paths below are this WSL machine's (see ARCHITECTURE.md) - adjust if yours differ.

REPO=~/sar-multidrone-coord
PX4=~/PX4-Autopilot
AGENT=~/Micro-XRCE-DDS-Agent-243/build/MicroXRCEAgent
SESSION=sim
RUNS=~/sim_runs  # `down` saves each run's logs here, plus a line in summary.txt
MISSION_LOG="$RUNS/current_mission.jsonl"  # sim.sh demo records here while running
RISK_MAP="$RUNS/current_risk_map.json"  # sim.sh demo-flood's flood map while running
TRUTH_LOG="$RUNS/current_truth.jsonl"  # true drone positions, if the backend logs them
VIDEO_DIR=/mnt/c/Users/UAV_group/Videos  # sim.sh render copies the GIF here (Windows side)
SELF="$(realpath "$0")"
ROS_ENV="source /opt/ros/humble/setup.bash && source $REPO/ros2_ws/install/setup.bash"
NODE_ARGS="-p num_drones:=2 -p use_px4_offboard:=true -p hold_altitude:=3.0"
NODE_ARGS+=" -p area_center_x:=4.0 -p area_center_y:=4.0 -p area_radius:=6.0"

say() { echo "[sim] $*"; }

source "$(dirname "$SELF")/fleet.sh"  # SPAWN_X / SPAWN_Y

SIM=${SIM:-gazebo}
BACKEND="$(dirname "$SELF")/backends/$SIM.sh"
[ -f "$BACKEND" ] || { say "unknown SIM=$SIM (no $BACKEND)"; exit 1; }
# shellcheck source=backends/gazebo.sh
source "$BACKEND"

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
  if [ -f "$MISSION_LOG" ]; then  # sim.sh demo's recording, for sim.sh render
    tmux send-keys -t "$SESSION:recorder" C-c 2>/dev/null
    sleep 1
    mv "$MISSION_LOG" "$dir/mission.jsonl"
    [ -f "$RISK_MAP" ] && mv "$RISK_MAP" "$dir/risk_map.json"
    say "mission recording saved - turn it into a video with: bash $SELF render"
  fi
  # Simulator ground truth, from backends that log it (AirSim's distance
  # watch) - compare with mission.jsonl's believed positions.
  [ -f "$TRUTH_LOG" ] && mv "$TRUTH_LOG" "$dir/truth.jsonl"
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
  backend_stop; pkill -f MicroXRCEAgent; pkill -f "px4_sitl_default/bin/px4"
  pkill -f coordination_node; pkill -f "ros2 topic pub"
  sleep 2
  say "stopped."
}

up() {
  local headless="" n env
  [ "${1:-}" = "--headless" ] && headless="--headless"
  down
  tmux new-session -d -s "$SESSION" -n shell -x 220 -y 50

  say "starting $BACKEND_NAME (shared world - must be up before any PX4)..."
  backend_world_up $headless || return 1

  # Gazebo needs each PX4 up before the next joins its world; AirSim may
  # not step its clock until every vehicle has connected, so there all
  # PX4s start first (BACKEND_PX4_TOGETHER).
  for n in 0 1; do
    say "starting PX4 instance $n..."
    env=$(backend_px4_env "$n" "${SPAWN_X[n]}" "${SPAWN_Y[n]}") || return 1
    window "px4_$n" "cd $PX4 && PX4_UXRCE_DDS_PORT=$((8888 + n)) $env ./build/px4_sitl_default/bin/px4 -i $n"
    [ -n "${BACKEND_PX4_TOGETHER:-}" ] && { sleep 2; continue; }
    wait_for "px4_$n" 'pxh>' 90 || { say "PX4 $n didn't start - look at window px4_$n."; return 1; }
  done
  for n in 0 1; do
    wait_for "px4_$n" 'pxh>' 90 || { say "PX4 $n didn't start - look at window px4_$n."; return 1; }
    window "agent$n" "$AGENT udp4 -p $((8888 + n))"
  done

  # Runtime-only PX4 settings, needed after every launch (ARCHITECTURE.md
  # "Required in each PX4 shell after every launch").
  for n in 0 1; do
    tmux send-keys -t "$SESSION:px4_$n" "param set NAV_DLL_ACT 0" Enter
    backend_px4_setup "$n"
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

node() {  # node N [EXTRA ROS ARGS for this drone only] - starts at its spawn point
  window "coord$1" "$ROS_ENV && ros2 run coordination_node coordination_node --ros-args -p drone_id:=$1 -p initial_x:=${SPAWN_X[$1]} -p initial_y:=${SPAWN_Y[$1]} $NODE_ARGS ${NODE_EXTRA:-} ${2:-}"
}

distance_window() {
  mkdir -p "$RUNS" && rm -f "$TRUTH_LOG"
  window dist "SIM=$SIM TRUTH_LOG=$TRUTH_LOG bash $REPO/ros2_ws/tools/drone_distance.sh"
}

wait_airborne() {  # wait_airborne N
  say "waiting for drone $1 to take off..."
  wait_for "px4_$1" 'Takeoff detected' 90 \
    || { say "drone $1 never took off - look at windows px4_$1 / coord$1."; return 1; }
  sleep 4  # finish climbing before handing out targets
}

detect() {  # detect X Y [N] [TYPE]
  say "target at ($1, $2), detected by drone ${3:-0}${4:+, type $4}"
  bash -c "$ROS_ENV && ros2 service call /drone_${3:-0}/coordination/detect_target coordination_msgs/srv/DetectTarget '{position: {x: $1, y: $2, z: 0.0}, target_type: ${4:-person}, confidence: 0.9}'" | tail -1
}

marker() {  # marker X Y [R G B] - visual-only, shared frame
  backend_marker "$1" "$2" "${3:-1}" "${4:-0}" "${5:-0}" \
    || say "marker at ($1, $2) failed (cosmetic only - carrying on)"
}

test_b() {
  up "$@" || return 1
  tmux set-environment -t "$SESSION" RUN_NAME test-b
  node 0
  node 1
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
  node 0
  node 1
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
  node 0
  node 1
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

demo() {  # the whole hybrid approach in one recorded run, for presenting
  up "$@" || return 1
  tmux set-environment -t "$SESSION" RUN_NAME demo
  mkdir -p "$RUNS"
  window recorder "$ROS_ENV && python3 $REPO/ros2_ws/tools/mission_recorder.py --num-drones 2 --out $MISSION_LOG"
  node 0
  node 1
  distance_window
  wait_airborne 0 && wait_airborne 1 || return 1

  say "act 1 - PSO search: 30s of both drones covering the area..."
  sleep 30

  say "act 2 - CBBA auction: 4 targets in two clusters (expect one cluster per drone)."
  marker 0 8 1 0 0; marker 1 9 1 0 0; marker 8 0 0 1 0; marker 9 1 0 1 0
  detect 0 8 0; detect 1 9 0; detect 8 0 0; detect 9 1 0
  sleep 30

  say "act 3 - resilience: 2 new targets, then drone 1's battery hits reserve."
  marker 9 6 1 0 1; marker -1 3 1 0 1
  detect 9 6 0; detect -1 3 0
  tmux send-keys -t "$SESSION:px4_1" "param set SIM_BAT_DRAIN 1" Enter
  tmux send-keys -t "$SESSION:px4_1" "param set SIM_BAT_MIN_PCT 20" Enter
  sleep 30
  say "demo done. Now: bash $SELF down   then   bash $SELF render"
}

demo_flood() {  # search weighted by a synthetic flood-risk map, recorded
  up "$@" || return 1
  tmux set-environment -t "$SESSION" RUN_NAME demo-flood
  mkdir -p "$RUNS"
  python3 "$REPO/ros2_ws/tools/make_risk_map.py" "$RISK_MAP" || return 1
  window recorder "$ROS_ENV && python3 $REPO/ros2_ws/tools/mission_recorder.py --num-drones 2 --out $MISSION_LOG"
  NODE_EXTRA="-p risk_map_file:=$RISK_MAP"
  node 0
  node 1
  distance_window
  wait_airborne 0 && wait_airborne 1 || return 1

  say "act 1 - flood-weighted search: a river bends across the north of the area;"
  say "the drones should go there first and keep coming back to it (45s)..."
  sleep 45
  say "act 2 - targets on the river bank and in the dry south (CBBA as usual)."
  marker 8 3 1 0 0; marker 1 1 0 1 0
  detect 8 3 0; detect 1 1 0
  sleep 30
  say "demo done. Now: bash $SELF down   then   bash $SELF render"
}

render() {  # render [RUN_DIR] - newest run with a mission recording by default
  local dir=${1:-$(ls -td "$RUNS"/*/ 2>/dev/null | while read -r d; do
    [ -f "$d/mission.jsonl" ] && { echo "${d%/}"; break; }; done)}
  [ -f "$dir/mission.jsonl" ] || { say "no mission.jsonl found - run: bash $SELF demo"; return 1; }
  say "rendering $dir/mission.jsonl (takes about a minute)..."
  python3 "$REPO/ros2_ws/tools/render_mission.py" "$dir/mission.jsonl" "$dir/mission.gif" \
    --area 4 4 6 $([ -f "$dir/risk_map.json" ] && echo --risk-map "$dir/risk_map.json") || return 1
  if [ -d "$VIDEO_DIR" ]; then
    cp "$dir/mission.gif" "$VIDEO_DIR/$(basename "$dir").gif"
    say "copied to Windows: C:\\Users\\UAV_group\\Videos\\$(basename "$dir").gif"
  fi
}

test_battery() {
  up "$@" || return 1
  tmux set-environment -t "$SESSION" RUN_NAME test-battery
  # PX4 SITL simulates the battery: SIM_BAT_DRAIN = seconds from full to
  # empty while armed, SIM_BAT_MIN_PCT = where it stops. A fast drain makes
  # drone 1's battery follow MIN_PCT within seconds of each change.
  tmux send-keys -t "$SESSION:px4_1" "param set SIM_BAT_DRAIN 20" Enter
  node 0
  node 1
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
  say "fly home to (5, 5) and land; drone 0 takes the rest (it holds 3 at a time)."
  tmux send-keys -t "$SESSION:px4_1" "param set SIM_BAT_MIN_PCT 20" Enter
  say "watch: bash $SELF attach -> Ctrl+b w -> coord0 / coord1 / dist"
}

test_handoff() {
  up "$@" || return 1
  tmux set-environment -t "$SESSION" RUN_NAME test-handoff
  node 0
  node 1
  distance_window
  wait_airborne 0 && wait_airborne 1 || return 1
  say "both searching for 15s..."
  sleep 15
  # Both healthy, so drone 1 wins its share (one cluster each, as in
  # test-bids)... The clusters sit ~13m out from the search area's centre:
  # at test-bids' (0,8)/(8,0) drone 1 finished its pair in ~6s, before
  # its battery reached reserve (~8s in AirSim), so there was nothing to
  # hand off (2026-10-15; Gazebo needed four tries for the same reason).
  marker -5 13 1 0 0; marker -4 14 1 0 0; marker 13 -5 0 1 0; marker 14 -4 0 1 0
  detect -5 13 0; detect -4 14 0; detect 13 -5 0; detect 14 -4 0
  # ...then runs low on the way to them - right away and near-instantly
  # (1s full-to-empty).
  say "targets: red (-5,13)+(-4,14), green (13,-5)+(14,-4), far out on purpose."
  say "drone 1 battery -> 20% mid-flight: it should hand its targets to drone 0,"
  say "fly home to (5, 5) and land; drone 0 should end up doing all 4 (3 at a time)."
  tmux send-keys -t "$SESSION:px4_1" "param set SIM_BAT_DRAIN 1" Enter
  tmux send-keys -t "$SESSION:px4_1" "param set SIM_BAT_MIN_PCT 20" Enter
  say "watch: bash $SELF attach -> Ctrl+b w -> coord0 / coord1 / dist"
}

test_x() {  # both drones flying, paths crossing head-on, twice
  up "$@" || return 1
  tmux set-environment -t "$SESSION" RUN_NAME test-x
  # CBBA never picks crossing paths on distance alone (swapping two
  # crossing assignments is always shorter), so each drone may only take
  # its own target type: x0 for drone 0, x1 for drone 1.
  mkdir -p "$RUNS"
  window recorder "$ROS_ENV && python3 $REPO/ros2_ws/tools/mission_recorder.py --num-drones 2 --out $MISSION_LOG"
  node 0 "-p task_types:=x0"
  node 1 "-p task_types:=x1"
  distance_window
  wait_airborne 0 && wait_airborne 1 || return 1

  say "crossing 1: drone 0 -> red (9, 9), drone 1 -> green (-4, -4), head-on along the diagonal."
  marker 9 9 1 0 0; marker -4 -4 0 1 0
  detect 9 9 0 x0; detect -4 -4 1 x1
  wait_for coord0 'reached target' 60 && wait_for coord1 'reached target' 60 \
    || say "a drone didn't report reaching its target in 60s - carrying on."

  say "crossing 2: back the other way - drone 0 -> red (-1, -1), drone 1 -> green (10, 10)."
  marker -1 -1 1 0 0; marker 10 10 0 1 0
  detect -1 -1 0 x0; detect 10 10 1 x1
  say "pass: both reach their targets twice, closest distance stays >= 1.5m."
  say "watch: bash $SELF attach -> Ctrl+b w -> coord0 / coord1 / dist"
}

repeat_search() {  # repeat_search N SECONDS [TEST] - for measuring separation
  local n=${1:-3} secs=${2:-120} test=${3:-test-search} i before
  test=${test//-/_}
  mkdir -p "$RUNS" && touch "$RUNS/summary.txt"
  before=$(wc -l < "$RUNS/summary.txt")
  for ((i = 1; i <= n; i++)); do
    say "=== run $i of $n ==="
    "$test" --headless || say "run $i failed to start - its logs are saved too."
    sleep "$secs"
    down
  done
  say "closest drone-to-drone distance per run:"
  tail -n +"$((before + 1))" "$RUNS/summary.txt"
}

test_c() {
  local sx sy i
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

  # Tell drone 0 where drone 1 really is.
  read -r sx sy <<< "$(backend_pose 1)"
  [ -z "$sx" ] && { say "couldn't read drone 1's position from $BACKEND_NAME."; return 1; }
  say "drone 1 hovering at shared ($sx, $sy)"
  window drone1_pos "$ROS_ENV && ros2 topic pub -r 2 /drone_1/coordination/agent_state coordination_msgs/msg/AgentState \"{drone_id: 1, position: {x: $sx, y: $sy, z: 0.0}, best_position: {x: $sx, y: $sy, z: 0.0}, best_fitness: 0.0, state: 0}\""

  node 0
  distance_window
  wait_airborne 0 || return 1
  marker 10 10 0 1 0
  detect 10 10 0
  say "Test C running: drone 0 should curve around drone 1 and reach (10, 10);"
  say "closest distance should stay >= 1.5m. watch: bash $SELF attach -> Ctrl+b w -> dist"
}

case "${1:-}" in
  up) shift; up "$@" ;;
  nodes) node 0; node 1; distance_window; say "nodes started." ;;
  test-search) shift; test_search "$@" ;;
  test-bids) shift; test_bids "$@" ;;
  test-battery) shift; test_battery "$@" ;;
  test-handoff) shift; test_handoff "$@" ;;
  demo) shift; demo "$@" ;;
  demo-flood) shift; demo_flood "$@" ;;
  render) shift; render "$@" ;;
  repeat) shift; repeat_search "$@" ;;
  test-b) shift; test_b "$@" ;;
  test-c) shift; test_c "$@" ;;
  test-x) shift; test_x "$@" ;;
  detect) shift; detect "$@" ;;
  marker) shift; marker "$@" ;;
  distance) bash "$REPO/ros2_ws/tools/drone_distance.sh" ;;
  attach) tmux attach -t "$SESSION" ;;
  down) down ;;
  *) sed -n '2,/^# Paths below/p' "$SELF" | sed 's/^# \{0,1\}//' ;;
esac
