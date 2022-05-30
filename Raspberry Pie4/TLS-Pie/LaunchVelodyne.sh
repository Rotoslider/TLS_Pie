#!/usr/bin/env bash

# Turn on Velodyne Packets
source /opt/ros/noetic/setup.bash 
source /home/lipi/catkin_ws/devel/setup.bash 

echo "Starting Velodyne, please wait!"
roslaunch velodyne_pointcloud VLP16_points.launch


