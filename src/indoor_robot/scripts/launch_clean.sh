#!/bin/bash

# Kill old processes
killall -9 gzserver gzclient gazebo 2>/dev/null

# Wait
sleep 2

# Navigate
cd /mnt/d/ROS2_PROJECTS/autonomous_indoor_robot

# Source
source install/setup.bash

# Launch
ros2 launch indoor_robot gazebo.launch.py
