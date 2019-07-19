#!/bin/sh
#xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=~/udacity_ws/src/RoboND-HomeServiceRobot/worlds/U-world.world " &
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 3
#xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=~/udacity_ws/src/RoboND-HomeServiceRobot/worlds/U-World.yaml " &
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 3
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch "
