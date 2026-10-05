#!/bin/bash
tmux new-session -d -s agent0 'cd ~ && ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888'
tmux new-session -d -s agent1 'cd ~ && ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8889'
sleep 3
echo AGENTS
pgrep -af MicroXRCEAgent
echo PX4_0_TAIL
tmux capture-pane -t px4_0 -p | tail -5
echo PX4_1_TAIL
tmux capture-pane -t px4_1 -p | tail -5
