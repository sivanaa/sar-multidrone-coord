#!/bin/bash
# Live horizontal distance between two drones, plus the closest they've
# come since starting - for checking the MIN_SEPARATION_M hard floor (1.5m)
# during a real two-drone PX4 run. Positions are true simulator poses, in
# the fleet's shared frame (same coordinates as targets and node logs).
# Usage: [SIM=gazebo|airsim] [NUM_DRONES=N] bash drone_distance.sh [drone_a] [drone_b]
#        (default drones 0 and 1; with NUM_DRONES > 2 and no drones named,
#        the closest pair of the whole fleet; Ctrl+C to stop)
a=${1:-0}
b=${2:-1}
min=999
say() { echo "[distance] $*"; }
tools="$(dirname "$(realpath "$0")")"
source "$tools/fleet.sh"  # SPAWN_X / SPAWN_Y
source "$tools/backends/${SIM:-gazebo}.sh" || exit 1
if [ "$NUM_DRONES" -gt 2 ] && [ -z "${1:-}" ]; then
  if declare -F backend_distance_watch >/dev/null; then
    backend_distance_watch  # no drones named: the whole fleet (AirSim)
    exit
  fi
  # Whole fleet: every drone's pose each sample, the closest pair reported
  # (same "closest X m" wording - sim.sh down reads it).
  while true; do
    poses=""
    for n in $DRONES; do
      read -r x y <<< "$(backend_pose "$n")"
      [ -z "$x" ] && { poses=""; break; }
      poses+="$n $x $y"$'\n'
    done
    if [ -z "$poses" ]; then
      echo "waiting for all $NUM_DRONES drones in $BACKEND_NAME..."
      sleep 1
      continue
    fi
    line=$(printf '%s' "$poses" | awk -v m="$min" '
      { id[NR] = $1; x[NR] = $2; y[NR] = $3 }
      END {
        best = 1e9
        for (i = 1; i <= NR; i++) for (j = i + 1; j <= NR; j++) {
          d = sqrt((x[i] - x[j]) ^ 2 + (y[i] - y[j]) ^ 2)
          if (d < best) { best = d; bi = id[i]; bj = id[j] }
        }
        if (best < m) m = best
        printf "%.2f %s-%s %.2f", best, bi, bj, m
      }')
    read -r d pair min <<< "$line"
    warn=$(awk -v m="$min" 'BEGIN { if (m < 1.5) print "  <-- went under 1.5m floor" }')
    echo "closest pair drone$pair  distance $d m  closest $min m$warn"
    sleep 0.2
  done
fi
if declare -F backend_distance_watch >/dev/null; then
  backend_distance_watch "$a" "$b"  # backend has a faster way (AirSim)
  exit
fi
while true; do
  read -r ax ay <<< "$(backend_pose "$a")"
  read -r bx by <<< "$(backend_pose "$b")"
  if [ -z "$ax" ] || [ -z "$bx" ]; then
    echo "waiting for drones $a and $b in $BACKEND_NAME..."
  else
    d=$(awk -v ax="$ax" -v ay="$ay" -v bx="$bx" -v by="$by" \
      'BEGIN { printf "%.2f", sqrt((ax-bx)^2 + (ay-by)^2) }')
    min=$(awk -v d="$d" -v m="$min" 'BEGIN { print (d < m ? d : m) }')
    warn=$(awk -v m="$min" 'BEGIN { if (m < 1.5) print "  <-- went under 1.5m floor" }')
    printf 'drone%s (%.2f, %.2f)  drone%s (%.2f, %.2f)  distance %s m  closest %s m%s\n' \
      "$a" "$ax" "$ay" "$b" "$bx" "$by" "$d" "$min" "$warn"
  fi
  sleep 0.2  # 1s missed the closest moment of a 4 m/s head-on pass
done
