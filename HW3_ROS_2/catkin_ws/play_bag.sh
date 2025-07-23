#!/bin/bash
# 运行bag文件的脚本
echo "=== Playing bag file for cone analysis ==="
echo "Make sure the cone_analysis node is running..."
echo "Starting bag playback in 3 seconds..."
sleep 3

cd /home/simon/Desktop/Homeworks/HW3_ROS_2/catkin_ws
source devel/setup.bash

echo "Playing bag file: lidar_cone_side_&_slam_state.bag"
rosbag play -l src/fsd_common_msgs/bag/lidar_cone_side_\&_slam_state.bag

echo "Bag file playback completed!"
