# robotlinker_core

* travis: [![BuildStatus](https://travis-ci.org/robotlinker/robotlinker_core.svg?branch=master)](https://travis-ci.org/robotlinker/robotlinker_core)
* snap: [![Snap Status](https://build.snapcraft.io/badge/robotlinker/robotlinker_core.svg)](https://build.snapcraft.io/user/robotlinker/robotlinker_core)
## Prerequisite
### UR Simulator
* Download [UR Simulator](https://www.universal-robots.com/download/)
* Unzip 
* Install Oracle JDK8 [JDK installation] (https://www.digitalocean.com/community/tutorials/how-to-install-java-with-apt-get-on-ubuntu-16-04)
* In `install.sh`, replace `openjdk-6-jre` with  `openjdk-8-jre`
* Install `bash install.sh`
* change the following to be executable `sudo chmod +x filename.sh`
    * start-ursim.sh
    * starturcontrol.sh
    * stopurcontrol.sh
    * URControl
* launch simulator `bash start-ursim.sh`
* update Java if display issues are present

## ur_modern_driver
* publish robot rostopic
* send actuation command to robot
* run `roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=localhost` 

### AWS-IOT python lib
* `git clone https://github.com/aws/aws-iot-device-sdk-python.git`
* `cd aws-iot-device-sdk-python`
* `sudo python setup.py install`
* install other python libs if necessary
    
## aws_gateway
* Gateway listens to robot's state
* Gateway uploads robot's state to cloud
* Cloud processes the data and return a command
* Gateway receives command and send to robot to execute
* run `roslaunch aws_gateway aws_test.launch` (instead of run `bash start.sh`)
* change `aws_test.launch` if you have different certs, keys or aws accts
* it has been tested under Ubuntu 14.04 with ROS Indigo

