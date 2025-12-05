#!/bin/bash

# Kill any existing processes (ignore errors if none running)
pkill -f gazebo 2>/dev/null
pkill -f nav2 2>/dev/null
pkill -f human_detector 2>/dev/null
sleep 2

# Kill existing tmux session if it exists
tmux kill-session -t finalproj 2>/dev/null

# Create tmux session
tmux new-session -d -s finalproj

# ======== PANEL 1: GAZEBO (NO RVIZ) ========
tmux send-keys "
echo '=== STARTING GAZEBO ==='
source /opt/ros/jazzy/setup.bash
source /MRTP/install/setup.bash

sleep 2
ros2 launch gazeboenvs tb4_warehouse.launch.py use_rviz:=false
" C-m

# ======== PANEL 2: NAV2 ========
tmux split-window -v
tmux send-keys "
echo '=== STARTING NAV2 ==='
source /opt/ros/jazzy/setup.bash
source /MRTP/install/setup.bash

# Give Gazebo time to spawn robot and publish TF
sleep 25

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true
" C-m

# ======== PANEL 3: HUMAN DETECTOR ========
tmux split-window -h
tmux send-keys "
echo '=== BUILDING HUMAN DETECTOR ==='
source /opt/ros/jazzy/setup.bash
source /MRTP/install/setup.bash

cd /finalproj

# Build if needed
if [ ! -f install/human_detector/lib/human_detector/human_detector ]; then
    echo 'Building human_detector...'
    colcon build --packages-select human_detector
fi

source install/setup.bash

echo '=== WAITING FOR NAV2 TO FULLY ACTIVATE ==='
sleep 40   # <-- IMPORTANT FIX (Nav2 needs this)

echo '=== STARTING HUMAN DETECTOR ==='
ros2 run human_detector human_detector 2>&1 | tee ~/detector.log
" C-m


# ======== PANEL 4: MONITORING ========
tmux split-window -v
tmux send-keys "
echo '=== MONITORING SYSTEM ==='
source /opt/ros/jazzy/setup.bash
source /MRTP/install/setup.bash

sleep 30
clear

echo '==============================================='
echo '           SYSTEM STATUS MONITOR              '
echo '==============================================='

while true; do
    echo ''
    echo '--- Active Nodes ---'
    ros2 node list 2>/dev/null || echo 'No nodes yet'

    echo ''
    echo '--- Human Detections ---'
    ros2 topic echo /human_markers --once 2>/dev/null | grep -A 3 'position:' || echo 'No detections yet'

    echo ''
    echo '--- Robot Pose ---'
    ros2 topic echo /amcl_pose --once 2>/dev/null | grep -A 2 'position:' || echo 'No pose data'

    echo ''
    echo '--- Navigation Status ---'
    ros2 action list | grep navigate || echo 'No navigation actions'

    echo ''
    echo '==============================================='
    echo 'Press Ctrl+C to exit monitoring'
    echo ''

    sleep 5
done
" C-m

# Layout
tmux select-layout tiled

echo "==============================================="
echo "  All systems launching in tmux session 'finalproj'"
echo "  Attaching in 3 seconds..."
echo "==============================================="
sleep 3

tmux attach-session -t finalproj

