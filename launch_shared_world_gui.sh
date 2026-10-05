#!/bin/bash
tmux kill-session -t gzworld 2>/dev/null
cd ~/PX4-Autopilot
tmux new-session -d -s gzworld -x 220 -y 50 'cd ~/PX4-Autopilot && export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/sivan/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins" && python3 Tools/simulation/gz/simulation-gazebo'
sleep 6
tmux capture-pane -t gzworld -p | tail -20
echo GZ_PROCS
pgrep -af "gz sim"
