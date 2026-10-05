#!/bin/bash
tmux kill-server 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f MicroXRCEAgent 2>/dev/null
pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
sleep 2
echo cleaned
pgrep -af "gz sim|MicroXRCEAgent|px4_sitl_default"
echo done
