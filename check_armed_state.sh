#!/bin/bash
tmux send-keys -t px4_1 'commander status' Enter
sleep 1
tmux capture-pane -t px4_1 -p | tail -10
