#!/bin/sh

# xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find pick_objects)/map/muks.world" &
sleep 5
# xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch map_file:=$(rospack find pick_objects)/map/map.yaml" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"
