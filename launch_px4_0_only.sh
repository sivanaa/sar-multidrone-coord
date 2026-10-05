#!/bin/bash
cd ~/PX4-Autopilot
tmux new-session -d -s px4_0 -x 220 -y 50 'cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_SIMULATOR=gz PX4_GZ_MODEL_POSE="0,0" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 0'
sleep 8
tmux capture-pane -t px4_0 -p | tail -25
echo GZ_PROCS
pgrep -af "gz sim"
