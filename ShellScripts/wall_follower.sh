#!/bin/sh
#xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=~/udacity_ws/src/RoboND-HomeServiceRobot/worlds/U-World.world " &
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
#xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=~/udacity_ws/src/RoboND-HomeServiceRobot/turtlebot_simulator/turtlebot_gazebo/launch/includes/gmapping.launch.xml " &
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 3
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 3
xterm  -e  " rosrun wall_follower wall_follower  "
