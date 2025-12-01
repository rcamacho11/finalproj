Final Project – Human Detection and Navigation (ROS 2 Jazzy + MRTP)

This repository contains the finalproj ROS 2 workspace for the MRTP final project.
The goal of the project is to navigate a TurtleBot4 in the MRTP warehouse simulation and determine the positions of two human characters based on LiDAR scans, comparing live sensor data against the static map.

This workspace includes:

A custom ROS 2 package: human_detector

A launch script: run_all.sh

Integration with the MRTP navigation library

Use of Nav2 for autonomous movement and obstacle avoidance

This repository does not contain MRTP itself. Each user must clone and build MRTP separately.

1. Prerequisites

Each user must have the following installed on their machine or dev container:

Required Software

ROS 2 Jazzy

MRTP workspace cloned and built successfully
https://github.com/stefanocarpin/MRTP

tmux (used by run_all.sh)

Gazebo (Fortress or Garden, depending on MRTP version)

colcon build tools

MRTP Folder Requirement

Your system should have:

/opt/ros/jazzy
/MRTP
/finalproj


The MRTP folder must contain the workspace exactly as provided by the MRTP repository and must be successfully built:

cd /MRTP
colcon build --symlink-install


After building, you should have:

/MRTP/install/setup.bash


This is required for the final project to run correctly.

2. Installation

Clone this repository:

git clone https://github.com/rcamacho11/finalproj.git


Make sure your folder structure looks like:

/MRTP
/finalproj


Build the final project:

cd /finalproj
colcon build


Source the workspace:

source /finalproj/install/setup.bash

3. Workspace Structure
finalproj/
│
├── src/
│   └── human_detector/
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── src/
│           └── human_detector.cpp
│
├── run_all.sh
│
├── build/                (auto-generated)
├── install/              (auto-generated)
└── log/                  (auto-generated)

4. How to Run the Project

This project uses a tmux-based launch script that starts:

Gazebo + RViz with the MRTP warehouse

Nav2 bringup

The human_detector_node

Before running, ensure MRTP has been built.

Step 1: Make launcher executable
cd /finalproj
chmod +x run_all.sh

Step 2: Run the full system
./run_all.sh

What the script does

Panel 1:

source /opt/ros/jazzy/setup.bash
source /MRTP/install/setup.bash
ros2 launch gazeboenvs tb4_warehouse.launch.py use_rviz:=true


Panel 2:

source /opt/ros/jazzy/setup.bash
source /MRTP/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true


Panel 3:

source /opt/ros/jazzy/setup.bash
source /MRTP/install/setup.bash
source /finalproj/install/setup.bash
ros2 run human_detector human_detector_node


You should see:

Gazebo warehouse environment

RViz with map and robot visualization

The robot starting at x=2.12, y=-21.3, yaw=1.57

The robot beginning patrol behavior

Dynamic detection of humans based on laser scan differences

Nav2 navigation toward detected human positions

5. How the Detection Works

The map from /map is used as ground truth.

LaserScan data /scan is analyzed for obstacles that appear in free map cells.

Detected obstacles are clustered into "human candidates."

Each candidate becomes a navigation goal.

Nav2 is used to autonomously move to each detected human.

When humans are visited, the system marks them as completed.

Once two humans are visited, the mission is considered complete.

6. Troubleshooting
Nav2 goal aborted or rejected

Ensure:

You sourced MRTP before launching Nav2.

Gazebo is unpaused.

There is a valid /map and /amcl_pose.

Robot will not move

Verify:

ros2 topic echo /amcl_pose


If no data appears, AMCL is not running or localization didn't initialize.

run_all.sh shows "command not found"

Make sure it is executable:

chmod +x run_all.sh

7. Contributing

If multiple teammates are collaborating, use standard Git workflow:

git checkout -b feature-name
git add .
git commit -m "Message"
git push origin feature-name


Then create a pull request on GitHub.

8. License

This project is for UC Merced MRTP course use only.
MRTP is licensed separately under the creator’s terms.
