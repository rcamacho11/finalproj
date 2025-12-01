#!/bin/bash

# Create tmux session
tmux new-session -d -s finalproj

# ======== PANEL 1: GAZEBO + RVIZ ========
tmux send-keys "
source /opt/ros/jazzy/setup.bash;
source /MRTP/install/setup.bash;
ros2 launch gazeboenvs tb4_warehouse.launch.py use_rviz:=true
" C-m

# ======== PANEL 2: NAV2 BRINGUP ========
tmux split-window -v
tmux send-keys "
source /opt/ros/jazzy/setup.bash;
source /MRTP/install/setup.bash;
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true
" C-m

# ======== PANEL 3: YOUR FINAL PROJECT NODE ========
tmux split-window -h
tmux send-keys "
source /opt/ros/jazzy/setup.bash;
source /MRTP/install/setup.bash;
source /finalproj/install/setup.bash;
ros2 run human_detector human_detector_node
" C-m

# Attach
tmux attach

