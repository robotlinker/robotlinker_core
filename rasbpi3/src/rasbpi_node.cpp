/*
Rasbpi3 receive sensor data from arduino and spacenav 3D joystick, package the data and send to the host GUI
Version: 0.01
Date: 2016-12-24
Author: Liang Conghui
Copyright: Aitreat Pte. Ltd.
*/
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include "std_msgs/Int16.h"
#include "../include/rt_msgs/robotlinker_protocol.h"
#include "../include/rt_msgs/motor_control.h"
#include "../include/ur_msgs/RobotStateRTMsg.h"

int tactile_data = 0;
int length_data = 0;
int ultrasonic_data = 0;
bool EP_STOP1 = false;  //tactile sensor safety protection trigger
bool EP_STOP2 = false; //joystick safety protection trigger
std_msgs::String movej2home;
std_msgs::String joystick2robot;
float tcp_speed[6];
std::string speed_msg[6];
std::stringstream ss;

/****sensor callback functions***/
void tactileCallback(const std_msgs::Int16::ConstPtr& msg)
{
  tactile_data = msg->data;

  if (tactile_data >= 600 && tactile_data <= 700){
      //ROS_INFO("Danger!!! Manipulator now retrun to home pose!");
      EP_STOP1 = true;
  } else if (tactile_data >= 700){
      ROS_INFO("Emergency!!! Now triger the EP-STOP of UR5!");
      EP_STOP1 = true;
  }
  ROS_INFO("I heard from tactile sensor: [%i]", msg->data);
}

void potentiometerCallback(const std_msgs::Int16::ConstPtr& msg)
{
  //ROS_INFO("I heard from potentiometer sensor: [%i]", msg->data);
  length_data = msg->data;
}

void ultrasonicCallback(const std_msgs::Int16::ConstPtr& msg)
{
  ultrasonic_data = msg->data;
  //ROS_INFO("I heard from ultrasonic sensor: [%i]", msg->data);
  if (ultrasonic_data <= 10){
      //ROS_INFO("massage device is contacting with human body now!");
  }
}

/****joystick callback function***/
void joystickCallback(const geometry_msgs::Vector3::ConstPtr& msg){
  //ROS_INFO ("I received data from joystick [%f][%f][%f]",msg->x, msg->y, msg->z);
  float offset_x = msg->x;
  float offset_y = msg->y;
  float offset_z = msg->z;

  if (offset_z<=-0.6){
    EP_STOP2=true;
    ROS_INFO("I heard from joystick EP_STOP2: [%f]", msg->z);
  }

  tcp_speed[0] = offset_x;
  tcp_speed[1] = offset_y;
  tcp_speed[2] = offset_z;
}

void robotposUpdateCallback(const ur_msgs::RobotStateRTMsg::ConstPtr& msg){
  float robot_pos[5];
  int i;

  //ROS_INFO("I heard from UR5 robot controller");
  for (i=0; i<=5; i++)
  {
    robot_pos[i] = msg->tool_vector[i];             //tool_vector_actual.data(0);
  }
//  //robot_pos[0] = msg->tcp_speed[0];
//  //x = msg->q_target;
//  Q_EMIT posUpdate(robot_pos[0], robot_pos[1], robot_pos[2], robot_pos[3], robot_pos[4], robot_pos[5]);
}

int main(int argc, char **argv)
{
    int i;
    //Set up ROS.
    ros::init(argc, argv, "rasbpi_node");
    ROS_INFO("Start to receive sensor data from arduino uno!");
    ros::NodeHandle n;

    //recive data from sensors that have been conenected with arduino
    ros::Subscriber sub1 = n.subscribe("tactile_data", 1000, tactileCallback);
    ros::Subscriber sub2 = n.subscribe("length_data", 1000, potentiometerCallback);
    ros::Subscriber sub3 = n.subscribe("ultrasonic_data", 1000, ultrasonicCallback);
    //receive spacenav joystick signal
    ros::Subscriber joystick_sub = n.subscribe("spacenav/offset", 1000, joystickCallback);
    ros::Subscriber robotPos_sub = n.subscribe("robot_stateRT", 1000, robotposUpdateCallback);

    //send raw sensor
    ros::Publisher data_pub = n.advertise<rt_msgs::robotlinker_protocol>("pidata_up", 1000);
    //send movej to robot controller
    ros::Publisher script_pub = n.advertise<std_msgs::String>("ur_driver/URScript", 1000);

    ros::Rate loop_rate(100); //Data update frequency is 100 HZ
    for (i=0; i<=5; i++)
    {
      tcp_speed[i] = 0;
     }

    while (ros::ok())
     {
       if (EP_STOP1 || EP_STOP2){
          //double home_joint[6] = {0.78539815, -2.0943950667, 1.7453292222, -1.2217304556, -1.5707963, 0};
          movej2home.data = "movej([0.78539815, -2.0943950667, 1.7453292222, -1.2217304556, -1.5707963, 1.5707963], a=0.3962634015954636, v=0.2471975511965976)";
          //movej2home ="movej([0.78539815, -2.0943950667, 1.7453292222, -1.2217304556, -1.5707963, 1.5707963], a=0.3962634015954636, v=0.2471975511965976)";
          script_pub.publish(movej2home);
          ROS_INFO("Move robot back to home pose");
          EP_STOP1 = false;
          EP_STOP2 = false;
         //EXT = true;
        }
       for (i=0; i<=5; i++)
       {
         ss << tcp_speed[i];
         speed_msg[i] = ss.str();
         //std::cout << tcp_speed[i] << "\n";
        }
       //chatter_publisher.publish(msg);

        joystick2robot.data ="speedl([" + speed_msg[0] + "," + speed_msg[1] + "," + speed_msg[2] + "," + speed_msg[3]+ "," + speed_msg[4] + "," + speed_msg[5] + "], 1.5, 2)";
        //joystick2robot.data = "speedj([0.78539815, -2.0943950667, 1.7453292222, -1.2217304556, -1.5707963, 1.5707963], a=0.3962634015954636, v=0.2471975511965976)";
        //script_pub.publish(joystick2robot);
        //ROS_INFO("Move robot using joystick commands");

        rt_msgs::robotlinker_protocol pidata_up;
        pidata_up.tactile_data = tactile_data;
        pidata_up.length_data = length_data;
        pidata_up.ultrasonic_data = ultrasonic_data;
        data_pub.publish(pidata_up);//publish received raw sensor data from arduino

        ros::spinOnce();
        loop_rate.sleep();
     }
    ros::spin();
    return 0;
}


