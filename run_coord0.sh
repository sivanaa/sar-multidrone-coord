#!/bin/bash
tmux kill-session -t coord0 2>/dev/null
tmux new-session -d -s coord0 -x 220 -y 50
tmux set-option -t coord0 remain-on-exit on
tmux send-keys -t coord0 'cd ~/sar-multidrone-coord/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run coordination_node coordination_node --ros-args -p drone_id:=0 -p num_drones:=2 -p initial_x:=0.0 -p initial_y:=0.0 -p use_px4_offboard:=true -p hold_altitude:=3.0 -p area_center_x:=4.0 -p area_center_y:=4.0 -p area_radius:=6.0; echo PROCESS_EXITED_CODE_$?' Enter
sleep 3
tmux capture-pane -t coord0 -p
