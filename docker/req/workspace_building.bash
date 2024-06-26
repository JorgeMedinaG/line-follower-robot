#!/bin/bash

EXEC_PATH=$PWD
echo "Execution path"
echo $EXEC_PATH
# mkdir -p ../turtlebot3_ws/src && cp -r ./followbot ../turtlebot3_ws/src/ && cd ../turtlebot3_ws/src

# git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
# git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
# git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
# git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
# git clone -b noetic-devel https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
# git clone https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning.git

cd $EXEC_PATH

cp ./docker/req/build.bash ../line_follower_ws/
#cp -r ../rcognita ../turtlebot3_ws/src/

echo "!!!DONE!!!"
