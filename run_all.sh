#!/bin/bash

echo "=== CLEANING OLD PROCESSES ==="
pkill -f gazebo 2>/dev/null
pkill -f nav2 2>/dev/null
pkill -f human_detector 2>/dev/null
sleep 2

tmux kill-session -t finalproj 2>/dev/null

# -------------------------
# CREATE TMUX SESSION
# -------------------------
tmux new-session -d -s finalproj

# =========================
# PANEL 1 — GAZEBO
# =========================
tmux send-keys "
echo '=== STARTING GAZEBO ==='
source /opt/ros/jazzy/setup.bash
source /MRTP/install/setup.bash
sleep 2
ros2 launch gazeboenvs tb4_warehouse.launch.py use_rviz:=false
" C-m

# =========================
# PANEL 2 — NAV2
# =========================
tmux split-window -v
tmux send-keys "
echo '=== STARTING NAV2 ==='
source /opt/ros/jazzy/setup.bash
source /MRTP/install/setup.bash
sleep 25
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true
" C-m

# =========================
# PANEL 3 — HUMAN DETECTOR
# =========================
tmux split-window -h
tmux send-keys "
echo '=== BUILDING HUMAN DETECTOR ==='
source /opt/ros/jazzy/setup.bash
source /MRTP/install/setup.bash

cd /finalproj

# Build if missing
if [ ! -f install/human_detector/lib/human_detector/human_detector ]; then
    echo 'Building human_detector...'
    colcon build --packages-select human_detector
fi

source install/setup.bash

echo '=== WAITING FOR NAV2 ==='
sleep 40

echo '=== STARTING HUMAN DETECTOR ==='
ros2 run human_detector human_detector 2>&1 | tee ~/detector.log
" C-m

# =========================
# PANEL 4 — LIVE MONITOR
# =========================
tmux split-window -v
tmux send-keys "
echo '=== MONITORING HUMANS DETECTED ==='
watch grep 'HUMAN' ~/detector.log
" C-m

tmux attach-session -t finalproj

