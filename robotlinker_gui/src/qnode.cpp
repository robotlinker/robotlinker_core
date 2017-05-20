/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/robotlinker_gui/qnode.hpp"
#include "../include/rt_msgs/robotlinker_protocol.h"
#include "../include/rt_msgs/motor_control.h"
#include "../include/ur_msgs/RobotStateRTMsg.h"
#include "../include/rt_msgs/ft_sensor.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace robotlinker_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {motorCmd_msg.data = "";
     motorCmd_update = false;}

QNode::~QNode() {
    if(ros::isStarted()) {
       ros::shutdown(); // explicitly needed since we use ros::start();
       ros::waitForShutdown();
    }
	wait();
}

void QNode::motorCmdPublisher(const std::string& param){
  std_msgs::String cmdStr;
  ROS_INFO("motor info");
  //cmdStr.data = msg.data;
   motorCmd_msg.data = param.data();
   motorCmd_update = true;
  //motor_cmd_pub.publish(cmdStr);
}


void QNode::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  QImage* img = new QImage(msg->width, msg->height, QImage::Format_RGB32);
  img->fill(QColor(Qt::white).rgb());

  for (int y = 0; y < msg->height; ++y) {
    for (int x = 0; x < msg->width; ++x) {
        img->setPixel(x, y, qRgb((int)msg->data[y*msg->step+x*3+2],
                                (int)msg->data[y*msg->step+x*3+1],
                                (int)msg->data[y*msg->step+x*3]));
    }
  }

  Q_EMIT imageUpdated(img);
}

void QNode::sensordataCallback(const rt_msgs::robotlinker_protocol::ConstPtr& msg)
{
  //ROS_INFO("I heard from potentiometer sensor: [%i] [%i] [%i]", msg->tactile_data, msg->length_data, msg->ultrasonic_data);
  Q_EMIT signalProgress(msg->tactile_data, msg->length_data, msg->ultrasonic_data);

}

void QNode::joystickCallback1(const geometry_msgs::Vector3::ConstPtr& msg){
  //ROS_INFO ("I received data from joystick [%f][%f][%f]",msg->x, msg->y, msg->z);
  joystickMovements_offset(msg->x, msg->y, msg->z);
  //joystickMovements(1.1, 2.0, 3.0);
}

void QNode::joystickCallback2(const geometry_msgs::Vector3::ConstPtr& msg){
  //ROS_INFO ("I received data from joystick [%f][%f][%f]",msg->x, msg->y, msg->z);
  joystickMovements_rot_offset(msg->x, msg->y, msg->z);
  //joystickMovements(1.1, 2.0, 3.0);
}

void QNode::ftSensorUpdateCallback(const rt_msgs::ft_sensor::ConstPtr& msg){
  rt_msgs::ft_sensor ft_sensor_data;

  ft_sensor_data.Fx = msg->Fx;
  ft_sensor_data.Fy = msg->Fy;
  ft_sensor_data.Fz = msg->Fz;
  ft_sensor_data.Mx = msg->Mx;
  ft_sensor_data.My = msg->My;
  ft_sensor_data.Mz = msg->Mz;
  Q_EMIT ftsensorUpdate(ft_sensor_data.Fx, ft_sensor_data.Fy, ft_sensor_data.Fz, ft_sensor_data.Mx, ft_sensor_data.My, ft_sensor_data.Mz);
}

void QNode::robotjointUpdateCallback(const sensor_msgs::JointState& msg)
{
  float joint_values[5];
  int i;

  for (i=0; i<=5; i++)
  {
   joint_values[i] = msg.position[i];             //tool_vector_actual.data(0);
  }
  //ROS_INFO("jOINT");
  Q_EMIT jointUpdate(joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4], joint_values[5]);
}


void QNode::robotposUpdateCallback(const ur_msgs::RobotStateRTMsg::ConstPtr& msg){
  float robot_pos[5];
  int i;

  //ROS_INFO("I heard from UR5 robot controller");
  for (i=0; i<=5; i++)
  {
    robot_pos[i] = msg->tool_vector[i];             //tool_vector_actual.data(0);
  }
  //robot_pos[0] = msg->tcp_speed[0];
  //x = msg->q_target;
  Q_EMIT posUpdate(robot_pos[0], robot_pos[1], robot_pos[2], robot_pos[3], robot_pos[4], robot_pos[5]);
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"robotlinker_gui");
	ros::start(); // explicitly needed since our nodehandle is going out of scope.

	ros::NodeHandle n;
  ROS_INFO("ROS Started!!!");

	// Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    data_sub = n.subscribe("pidata_up", 1000, &QNode::sensordataCallback, this);
    joystick_sub1 = n.subscribe("spacenav/offset", 1000, &QNode::joystickCallback1, this);
    joystick_sub2 = n.subscribe("spacenav/rot_offset", 1000, &QNode::joystickCallback2, this);
    robotPos_sub = n.subscribe("robot_stateRT", 1000, &QNode::robotposUpdateCallback, this);
    //robotJoint_sub = n.subscribe("joint_statesRT", 1000, &QNode::robotjointUpdateCallback, this);
    robotJoint_sub = n.subscribe("joint_states", 1000, &QNode::robotjointUpdateCallback, this);
    ftsensor_sub = n.subscribe("robotiq_force_torque_sensor", 1000, &QNode::ftSensorUpdateCallback, this);
    motor_cmd_pub = n.advertise<std_msgs::String>("motor_cmd", 1000);

    //image_transport::ImageTransport it(n);
    img_subscriber = n.subscribe("/kinect2/hd/image_color", 1, &QNode::imageCallback, this);
    //Listener listener;
    //ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::callback, &listener);
    start();
    return true;
}

void QNode::run() {
    ros::Rate loop_rate(10);
    int count = 0;
    ROS_INFO("HERE");
    while ( ros::ok() ) {
        //joystickMovements(count, count, count);
        //std_msgs::String msg;
        //std::stringstream ss;
        //ss << "hello world " << count;
        //msg.data = ss.str();
        //chatter_publisher.publish(msg);
        if (motorCmd_update){
           motor_cmd_pub.publish(motorCmd_msg);
           motorCmd_update = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


 } // namespace robotlinker_gui
