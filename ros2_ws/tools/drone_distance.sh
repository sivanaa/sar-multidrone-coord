#!/bin/bash
# Live horizontal distance between two Gazebo models (default: the two x500
# drones), plus the closest they've come since starting - for checking the
# MIN_SEPARATION_M hard floor (1.5m) during a real two-drone PX4 run.
# Usage: bash drone_distance.sh [model_a] [model_b]    (Ctrl+C to stop)
a=${1:-x500_0}
b=${2:-x500_1}
min=999
pose() { gz model -m "$1" -p 2>/dev/null | grep -A1 XYZ | tail -1 | tr -d '[]'; }
while true; do
  read -r ax ay _ <<< "$(pose "$a")"
  read -r bx by _ <<< "$(pose "$b")"
  if [ -z "$ax" ] || [ -z "$bx" ]; then
    echo "waiting for $a and $b in Gazebo..."
  else
    d=$(awk -v ax="$ax" -v ay="$ay" -v bx="$bx" -v by="$by" \
      'BEGIN { printf "%.2f", sqrt((ax-bx)^2 + (ay-by)^2) }')
    min=$(awk -v d="$d" -v m="$min" 'BEGIN { print (d < m ? d : m) }')
    warn=$(awk -v m="$min" 'BEGIN { if (m < 1.5) print "  <-- went under 1.5m floor" }')
    printf '%s (%.2f, %.2f)  %s (%.2f, %.2f)  distance %s m  closest %s m%s\n' \
      "$a" "$ax" "$ay" "$b" "$bx" "$by" "$d" "$min" "$warn"
  fi
  sleep 1
done
