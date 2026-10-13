# Gazebo (gz sim, Harmonic) backend for sim.sh and drone_distance.sh -
# sourced, not run. Everything sim.sh needs from a simulator goes through
# the backend_* functions below; airsim.sh implements the same ones.
#
# Frames: sim.sh hands every position in the fleet's shared frame (drone
# 0's PX4 local NED frame: x = north, y = east). Gazebo's world frame is
# ENU (x = east, y = north), so shared (x, y) is Gazebo (y, x) - see
# ARCHITECTURE.md "Shared coordinate frame".

BACKEND_NAME=gazebo

backend_world_up() {  # backend_world_up [--headless] - start the world, wait for it
  window gz "cd $PX4 && export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4/build/px4_sitl_default/src/modules/simulation/gz_plugins && python3 Tools/simulation/gz/simulation-gazebo ${1:-}"
  local i
  for ((i = 0; i < 120; i++)); do
    gz topic -l 2>/dev/null | grep -q '^/world/default/clock' && return 0
    sleep 1
  done
  say "Gazebo didn't come up in 120s - look at window gz (bash $SELF attach)."
  return 1
}

backend_px4_env() {  # backend_px4_env N X Y - env vars for PX4 instance N spawned at shared (X, Y)
  echo "PX4_SYS_AUTOSTART=4001 PX4_SIMULATOR=gz PX4_GZ_MODEL_POSE=$3,$2 PX4_GZ_MODEL=x500"
}

backend_px4_setup() {  # backend_px4_setup N - simulator-specific commands for PX4 shell N
  # gz's baro plugin didn't start on its own: "Preflight Fail: barometer
  # 0 missing" (2026-09-30).
  tmux send-keys -t "$SESSION:px4_$1" "sensor_baro_sim start" Enter
}

backend_marker() {  # backend_marker X Y R G B - visual-only marker at shared (X, Y)
  local name="marker_$(date +%s%N)"
  local sdf="<?xml version='1.0'?><sdf version='1.9'><model name='$name'><static>true</static><pose>$2 $1 0.5 0 0 0</pose><link name='l'><visual name='v'><geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry><material><ambient>$3 $4 $5 1</ambient><diffuse>$3 $4 $5 1</diffuse></material></visual></link></model></sdf>"
  gz service -s /world/default/create --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 3000 --req "sdf: \"$sdf\"" >/dev/null
}

backend_pose() {  # backend_pose N - drone N's true position as "X Y" in the shared frame
  local gx gy
  read -r gx gy _ <<< "$(gz model -m "x500_$1" -p 2>/dev/null | grep -A1 XYZ | tail -1 | tr -d '[]')"
  [ -n "$gx" ] && echo "$gy $gx"
}

backend_stop() {  # backend_stop - kill whatever backend_world_up started
  pkill -f "gz sim"
}
