#!/bin/bash

cd /line_follower_ws \
     && . /opt/ros/noetic/setup.sh \
     && catkin_make
   
exec bash

