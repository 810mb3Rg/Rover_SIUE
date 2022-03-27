#!/bin/sh

gnome-terminal --tab -- bash -ic "export TITLE_DEFAULT='lidar'; roslaunch urg_lidar.launch; exec bash;"
sleep 8
gnome-terminal --tab -- bash -ic "export TITLE_DEFAULT='gmapping'; roslaunch rover_gmapping_launch.launch; exec bash;"








