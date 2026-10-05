#!/bin/bash
cd ~/PX4-Autopilot
tmux new-session -d -s px4_1 -x 220 -y 50 'cd ~/PX4-Autopilot && PX4_UXRCE_DDS_PORT=8889 PX4_SYS_AUTOSTART=4001 PX4_SIMULATOR=gz PX4_GZ_MODEL_POSE="5,5" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1'
sleep 8
tmux capture-pane -t px4_1 -p -S -40
echo GZ_PROCS
pgrep -af "gz sim"
