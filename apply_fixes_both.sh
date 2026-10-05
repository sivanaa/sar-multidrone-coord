#!/bin/bash
tmux send-keys -t px4_0 'param set NAV_DLL_ACT 0' Enter
tmux send-keys -t px4_1 'param set NAV_DLL_ACT 0' Enter
sleep 1
tmux send-keys -t px4_0 'sensor_baro_sim start' Enter
tmux send-keys -t px4_1 'sensor_baro_sim start' Enter
sleep 1
tmux send-keys -t px4_0 'uxrce_dds_client status' Enter
tmux send-keys -t px4_1 'uxrce_dds_client status' Enter
sleep 1
echo PX4_0
tmux capture-pane -t px4_0 -p | tail -15
echo PX4_1
tmux capture-pane -t px4_1 -p | tail -15
