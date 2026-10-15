# AirSim backend for sim.sh and drone_distance.sh - sourced, not run.
# Same backend_* functions as gazebo.sh. AirSim (v1.8.1, Unreal 4.27,
# Blocks environment) runs on Windows; PX4, the DDS agents and the nodes
# stay in WSL and reach it over the WSL virtual network.
#
# Frames: AirSim's world frame is NED with its origin at PlayerStart, where
# drone 0 spawns, so shared (x, y) is AirSim (x, y) - no swap, unlike
# Gazebo. Details in airsim/airsim_tool.py.

BACKEND_NAME=airsim
BACKEND_PX4_TOGETHER=1  # see sim.sh up()
AIRSIM_DIR="$(dirname "$(realpath "${BASH_SOURCE[0]}")")/../airsim"
# Paths on this machine (see ARCHITECTURE.md) - adjust if yours differ.
AIRSIM_EDITOR=/mnt/d/UE_4.27/Engine/Binaries/Win64/UE4Editor.exe
AIRSIM_UPROJECT='D:\Sim\AirSim\Unreal\Environments\Blocks\Blocks.uproject'
AIRSIM_LOG=/mnt/d/Sim/AirSim/Unreal/Environments/Blocks/Saved/Logs/Blocks.log
AIRSIM_SETTINGS=/mnt/c/Users/UAV_group/Documents/AirSim/settings.json
# Windows as seen from WSL (WSL's default gateway). Changes when Windows
# restarts, so it's looked up every time rather than written down.
AIRSIM_HOST=$(ip route show default | awk '{print $3; exit}')

airsim_tool() { python3 "$AIRSIM_DIR/airsim_tool.py" "$@" --host "$AIRSIM_HOST"; }

backend_world_up() {  # backend_world_up [--headless] - start AirSim, wait for it
  local i spawns=() n
  for n in "${!SPAWN_X[@]}"; do spawns+=("--spawn=${SPAWN_X[n]},${SPAWN_Y[n]}"); done
  # Keep whatever settings.json was there before sim.sh first replaced it.
  [ -f "$AIRSIM_SETTINGS" ] && [ ! -f "$AIRSIM_SETTINGS.before-sim-sh" ] \
    && cp "$AIRSIM_SETTINGS" "$AIRSIM_SETTINGS.before-sim-sh"
  airsim_tool settings "${spawns[@]}" ${1:+--headless} ${CAMERA:+--camera "$CAMERA"} \
    > "$AIRSIM_SETTINGS" \
    || { say "couldn't write $AIRSIM_SETTINGS"; return 1; }
  window airsim "'$AIRSIM_EDITOR' '$AIRSIM_UPROJECT' -game -windowed -ResX=1280 -ResY=720 -log -unattended ${1:+-RenderOffScreen}"
  # Ready once AirSim listens for PX4 on TCP 4560. Its startup blocks there
  # (MavLinkMultirotorApi acceptTcp) until PX4 connects, so its API port
  # isn't up yet, and the "Waiting for TCP connection" line goes to the
  # screen, not the log. Asked of Windows, not probed: connecting to 4560
  # would be accepted as drone 0's PX4 link.
  for ((i = 0; i < 90; i++)); do
    powershell.exe -NoProfile -Command "if (Get-NetTCPConnection -LocalPort 4560 -State Listen -ErrorAction SilentlyContinue) { exit 0 } else { exit 1 }" >/dev/null 2>&1 \
      && return 0
    sleep 2
  done
  say "AirSim isn't listening for PX4 after 180s - look at its window, or $AIRSIM_LOG."
  return 1
}

backend_px4_env() {  # backend_px4_env N X Y - env vars for PX4 instance N
  # Airframe 10016 (none_iris) runs PX4's simulator_mavlink bridge, which
  # dials AirSim on TCP 4560 + N; the spawn point lives in settings.json.
  echo "PX4_SYS_AUTOSTART=10016 PX4_SIM_HOST_ADDR=$AIRSIM_HOST"
}

backend_px4_setup() { :; }  # nothing extra: AirSim brings its own sensors

backend_marker() {  # backend_marker X Y R G B - visual-only marker at shared (X, Y)
  airsim_tool marker "$@" >/dev/null 2>&1
}

backend_pose() {  # backend_pose N - drone N's true position as "X Y", shared frame
  airsim_tool pose "$1" --spawn-x "${SPAWN_X[$1]}" --spawn-y "${SPAWN_Y[$1]}" 2>/dev/null
}

backend_stop() {  # backend_stop - close the AirSim (Blocks) window, if sim.sh opened one
  powershell.exe -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='UE4Editor.exe'\" | Where-Object { \$_.CommandLine -match 'Blocks.uproject' } | ForEach-Object { Stop-Process -Id \$_.ProcessId -Force }" >/dev/null 2>&1
}

backend_distance_watch() {  # backend_distance_watch A B - optional: drone_distance.sh
  # uses this instead of polling backend_pose, which costs a fresh Python
  # process per call here (~1.2 samples/s); this keeps one API connection.
  local spawns=() n
  for n in "${!SPAWN_X[@]}"; do spawns+=("--spawn=${SPAWN_X[n]},${SPAWN_Y[n]}"); done
  airsim_tool watch "$1" "$2" "${spawns[@]}" ${TRUTH_LOG:+--log "$TRUTH_LOG"}
}

backend_camera_logger() {  # backend_camera_logger OUT_DIR - optional: prints the
  # command sim.sh runs (in a ROS-sourced window) for CAMERA=WxH runs.
  local spawns="" n
  for n in "${!SPAWN_X[@]}"; do spawns+=" --spawn=${SPAWN_X[n]},${SPAWN_Y[n]}"; done
  echo "python3 $AIRSIM_DIR/frame_logger.py --host $AIRSIM_HOST$spawns --out $1"
}

backend_target() {  # backend_target NAME X Y - optional: a real target object at
  # shared (X, Y) that cameras can see (vs backend_marker, drawn on screen
  # only). Prints its settled position as one JSON line.
  airsim_tool target "$1" "$2" "$3"
}

backend_vision_bridge() {  # backend_vision_bridge LOG - optional: prints the
  # command for the camera -> detect_target bridge (sim.sh test-vision).
  local spawns="" n
  for n in "${!SPAWN_X[@]}"; do spawns+=" --spawn=${SPAWN_X[n]},${SPAWN_Y[n]}"; done
  echo "python3 $AIRSIM_DIR/vision_bridge.py --host $AIRSIM_HOST$spawns --log $1"
}
