#!/bin/bash

cd
cd robotlinker_ws
source devel/setup.bash
cd ..
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.0.50
