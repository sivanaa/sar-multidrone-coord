#!/bin/bash
# Re-run every simulator experiment behind the report's figures, writing
# raw per-mission / per-problem results to results/data/*.json, which
# tools/make_plots.py turns into results/figures/*.svg. Same settings as
# the tables in ARCHITECTURE.md "Next steps" 8.
#   bash ros2_ws/tools/report_sweeps.sh [parallel jobs, default 8]
# About an hour on 8 cores. Skips any result file that already exists, so
# an interrupted run can just be started again.
cd "$(dirname "$(realpath "$0")")" || exit 1
out="../../results/data"
mkdir -p "$out"
jobs=${1:-8}

# "Plain CBBA": every CBBA change of 2026-10-16 off (re-broadcast,
# duplicate merging, outbid margin, bid refresh, 1m task arrival radius).
plain="--set brain.REBROADCAST_S=0 --set brain.CBBA_DUPLICATE_RADIUS_M=0 \
--set brain.CBBA_OUTBID_MARGIN=0 --set brain.CBBA_REFRESH_BIDS=False \
--set brain.TASK_ARRIVAL_RADIUS_M=0.3"
big="--area 4 4 12 --max-time 500 --trials 60 --targets 4"

{
  # CBBA against the brute-force optimum (static problems)
  for nm in "2 4" "3 5" "4 6"; do set -- $nm
    echo "opt_n$1 python3 cbba_optimality.py --drones $1 --tasks $2 --trials 300 --area 4 4 15"
  done
  # CBBA under message loss, plain vs ours (6m test area)
  for n in 3 4; do for loss in 0 0.2 0.4 0.6; do
    echo "loss_ours_n${n}_l$loss python3 fleet_sim.py --drones $n --targets 4 --trials 80 --max-time 200 --loss $loss"
    echo "loss_plain_n${n}_l$loss python3 fleet_sim.py --drones $n --targets 4 --trials 80 --max-time 200 --loss $loss $plain"
  done; done
  # Search methods, and the drone-leaving / river scenarios (12m area)
  for n in 2 3 4; do
    for s in pso greedy lawnmower random; do
      echo "search_base_${s}_n$n python3 fleet_sim.py --drones $n $big --search $s"
    done
    for sc in "dropout --dropout 15" "river --river 0.8" "both --dropout 15 --river 0.8"; do
      set -- $sc; name=$1; shift
      for s in pso lawnmower; do
        echo "search_${name}_${s}_n$n python3 fleet_sim.py --drones $n $big --search $s $*"
      done
      # the same with the old 60s revisit time
      echo "revisit60_${name}_n$n python3 fleet_sim.py --drones $n $big --revisit 60 $*"
    done
    echo "revisit60_base_n$n python3 fleet_sim.py --drones $n $big --revisit 60"
  done
} | while read -r name cmd; do
  [ -s "$out/$name.json" ] || echo "$name $cmd --json $out/$name.json"
done | xargs -P "$jobs" -L 1 bash -c '"$@" > /dev/null && echo "done $0"'
echo "all results in $out"
