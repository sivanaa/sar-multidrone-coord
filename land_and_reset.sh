#!/bin/bash
tmux send-keys -t px4_1 'commander land' Enter
sleep 5
tmux capture-pane -t px4_1 -p | tail -10
