#!/bin/bash
export ROS_IP=192.168.1.200
export ROS_MASTER_URI=http://192.168.1.200:11311
sleep 2
rosrun rover sheep_marker.py
