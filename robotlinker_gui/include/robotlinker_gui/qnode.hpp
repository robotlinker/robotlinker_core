/**
 * @file /include/robotlinker_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robotlinker_gui_QNODE_HPP_
#define robotlinker_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QImage>
#include <image_transport/image_transport.h>
#include <QStringListModel>
#include "../include/rt_msgs/robotlinker_protocol.h"
#include "../include/rt_msgs/motor_control.h"
#include "../include/ur_msgs/RobotStateRTMsg.h"
#include "../include/rt_msgs/ft_sensor.h"
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <QString>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotlinker_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();
    //void emit50(){ Q_EMIT signalProgress(50); }
    void sensordataCallback(const rt_msgs::robotlinker_protocol::ConstPtr& msg);
    void joystickCallback1(const geometry_msgs::Vector3::ConstPtr& msg);
    void joystickCallback2(const geometry_msgs::Vector3::ConstPtr& msg);
    void robotposUpdateCallback(const ur_msgs::RobotStateRTMsg::ConstPtr& msg);
    void robotjointUpdateCallback(const sensor_msgs::JointState& msg);
    void ftSensorUpdateCallback(const rt_msgs::ft_sensor::ConstPtr& msg);
    //void motorCmdPublisher(const std_msgs::String& msg);
    void motorCmdPublisher(const std::string& param);


Q_SIGNALS:
    void rosShutdown();
    void signalProgress(int, int, int);
    void joystickMovements_offset(float, float, float);
    void joystickMovements_rot_offset(float, float, float);
    void posUpdate(double, double, double, double, double, double);
    void jointUpdate(double, double, double, double, double, double);
    void ftsensorUpdate(double, double, double, double, double, double);
    void imageUpdated(QImage*);

private:
    int init_argc;
    char** init_argv;
    ros::Publisher chatter_publisher;
    ros::Subscriber data_sub;
    ros::Subscriber joystick_sub1;
    ros::Subscriber joystick_sub2;
    ros::Subscriber robotPos_sub;
    ros::Subscriber robotJoint_sub;
    ros::Publisher motor_cmd_pub;
    ros::Subscriber ftsensor_sub;
    std_msgs::String motorCmd_msg;
    bool motorCmd_update;
    //image_transport::Subscriber img_subscriber;
    ros::Subscriber img_subscriber;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};




}  // namespace robotlinker_gui

#endif /* robotlinker_gui_QNODE_HPP_ */
