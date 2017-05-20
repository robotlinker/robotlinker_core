/**
 * @file /include/robotlinker_gui/main_window.hpp
 *
 * @brief Qt based gui for robotlinker_gui.
 *
 * @date November 2010
 **/
#ifndef robotlinker_gui_MAIN_WINDOW_H
#define robotlinker_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QString>

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robotlinker_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
  void on_sensor_data_update(int, int, int);
  void on_joystick_offset_data_update(float, float, float);
  void on_joystick_rot_offset_data_update(float, float, float);
  void on_robot_pos_update(double, double, double, double, double, double);
  void on_robot_joint_update(double, double, double, double, double, double);
  void on_ft_sensor_update(double, double, double, double, double, double);
  void on_motor_calibration_clicked();
  void on_motor_forward_clicked();
  void on_motor_backward_clicked();
  void on_motor_control_clicked();
  void cameraViewUpdate(QImage*);
  void on_pushButton_raw_image_clicked();
  void on_pushButton_depth_image_clicked();
  void on_pushButton_pcl_data_clicked();


private:
	Ui::MainWindowDesign ui;
	QNode qnode;

  QGraphicsScene *scene;
  QGraphicsEllipseItem *ellipse;
  QGraphicsRectItem *rectangle;
  QGraphicsTextItem *text;

};

}  // namespace robotlinker_gui

#endif // robotlinker_gui_MAIN_WINDOW_H
