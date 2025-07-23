#!/bin/bash
# 运行锥桶分析节点

echo "=== Starting Cone Analysis Node ==="
cd /home/simon/Desktop/Homeworks/HW3_ROS_2/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

echo "Environment setup complete"
echo "Starting cone counter and visualizer..."

export PYTHONPATH=$PYTHONPATH:/home/simon/Desktop/Homeworks/HW3_ROS_2/catkin_ws/devel/lib/python3/dist-packages

roslaunch cone_analysis cone_analysis.launch
