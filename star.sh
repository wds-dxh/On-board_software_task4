#!/bin/bash

# 关闭所有ROS节点
killall rosmaster
killall roslaunch
killall rosrun
echo "All existing ROS nodes have been terminated."
# 给工作空间设置环境一些时间
sleep 2
# 启动ROS Master
gnome-terminal -- bash -c "roscore; exec bash"
echo "Started ROS Master in a new terminal window."
# 给工作空间设置环境一些时间
sleep 2
# 进入工作空间并设置环境
gnome-terminal -- bash -c "cd /home/agilex/wds/task4_pub_sub && source devel/setup.bash; exec bash"

# 给工作空间设置环境一些时间
sleep 2

# 启动相机节点
gnome-terminal -- bash -c "cd /home/agilex/wds/task4_pub_sub && source devel/setup.bash && roslaunch realsense2_camera rs_camera.launch; exec bash"
echo "Started RealSense Camera Node in a new terminal window."
# 给工作空间设置环境一些时间
sleep 2
# 启动雷达节点
gnome-terminal -- bash -c "cd /home/agilex/wds/task4_pub_sub && source devel/setup.bash && roslaunch scout_bringup open_rslidar.launch; exec bash"
echo "Started Radar Node in a new terminal window."
# 给工作空间设置环境一些时间
sleep 2
# 启动目标发布节点
gnome-terminal -- bash -c "cd /home/agilex/wds/task4_pub_sub && source devel/setup.bash && rosrun nav_goal_publisher_subscriber nav_goal_publisher_subscriber_node; exec bash"
echo "Started Goal Publisher Node in a new terminal window."
# 给工作空间设置环境一些时间
sleep 2
# 启动导航节点
gnome-terminal -- bash -c "cd /home/agilex/wds/task4_pub_sub && source devel/setup.bash && roslaunch scout_bringup navigation_4wd.launch; exec bash"
echo "Started Navigation Node in a new terminal window."
# 给工作空间设置环境一些时间
sleep 2
# 启动识别节点
gnome-terminal -- bash -c "cd /home/agilex/wds/task4_pub_sub && source devel/setup.bash && rosrun tesseract_image_txt tesseract_image_txt_node ; exec bash"
echo "Started Tesseract Image Recognition Node in a new terminal window."