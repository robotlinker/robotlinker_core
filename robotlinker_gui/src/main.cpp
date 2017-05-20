/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui>
#include <QApplication>
#include "../include/robotlinker_gui/main_window.hpp"
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    //int code = system("~/robotlinker_ws/src/robotlinker_core/robotlinker_gui/script/roscore_start.sh &");
        //code = system("~/robotlinker_ws/src/robotlinker_core/robotlinker_gui/script/joystick_start.sh &");
        //code = system("~/robotlinker_ws/src/robotlinker_core/script/arduino_start.sh &");
        //code = system("~/robotlinker_ws/src/robotlinker_core/robotlinker_gui/script/connect_robot.sh &");
    robotlinker_gui::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

    int code = system("~/robotlinker_ws/src/robotlinker_core/robotlinker_gui/script/ros_kill.sh &");

	return result;
}
