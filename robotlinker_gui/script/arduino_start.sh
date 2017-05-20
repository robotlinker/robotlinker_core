#!/bin/bash

cd
cd robotlinker_ws
source devel/setup.bash
cd ..
rosrun rosserial_python serial_node.py /dev/ttyACM0
