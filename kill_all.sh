#!/bin/bash

# 关闭所有ROS节点
killall rosmaster
killall roslaunch
killall rosrun
echo "All existing ROS nodes have been terminated."
# 给工作空间设置环境一些时间
sleep 2