#!/bin/bash
# Live horizontal distance between two drones, plus the closest they've
# come since starting - for checking the MIN_SEPARATION_M hard floor (1.5m)
# during a real two-drone PX4 run. Positions are true simulator poses, in
# the fleet's shared frame (same coordinates as targets and node logs).
# Usage: [SIM=gazebo|airsim] bash drone_distance.sh [drone_a] [drone_b]
#        (default drones 0 and 1; Ctrl+C to stop)
a=${1:-0}
b=${2:-1}
min=999
say() { echo "[distance] $*"; }
source "$(dirname "$(realpath "$0")")/backends/${SIM:-gazebo}.sh" || exit 1
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
