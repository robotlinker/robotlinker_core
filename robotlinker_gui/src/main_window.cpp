/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <sstream>
#include <std_msgs/String.h>
#include "../include/robotlinker_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotlinker_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    //ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    //ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));


    ui.progressBar->setMinimum(0);
    ui.progressBar->setMaximum(1023);
    //ui.progressBar->setValue(0);
    //QObject::connect( &qnode, SIGNAL( signalProgress(int) ), ui.progressBar, SLOT( setValue(int) ) );
    QObject::connect( &qnode, SIGNAL( signalProgress(int, int, int) ), this, SLOT( on_sensor_data_update(int, int, int) ) );
    QObject::connect( &qnode, SIGNAL( joystickMovements_offset(float, float, float) ), this, SLOT( on_joystick_offset_data_update(float, float, float) ) );
    QObject::connect( &qnode, SIGNAL( joystickMovements_rot_offset(float, float, float) ), this, SLOT( on_joystick_rot_offset_data_update(float, float, float) ) );
    QObject::connect( &qnode, SIGNAL( posUpdate(double, double, double, double, double, double) ), this, SLOT(on_robot_pos_update(double, double, double, double, double, double) ) );
    QObject::connect( &qnode, SIGNAL( jointUpdate(double, double, double, double, double, double) ), this, SLOT(on_robot_joint_update(double, double, double, double, double, double) ) );
    QObject::connect( &qnode, SIGNAL( ftsensorUpdate(double, double, double, double, double, double) ), this, SLOT(on_ft_sensor_update(double, double, double, double, double, double) ) );

    //ui.lineEdit->setText(QString::number(0, 'g',3));
    //ui.lineEdit_2->setText(QString::number(0, 'g',3));
    //ui.lineEdit_3->setText(QString::number(0, 'g',3));
    ui.progressBar_2->setMinimum(0);
    ui.progressBar_2->setMaximum(100);
    ui.progressBar_2->setValue(0);
    ui.progressBar_3->setMinimum(0);
    ui.progressBar_3->setMaximum(30);
    ui.progressBar_3->setValue(0);

    QPixmap image(":/images/icon.png");
    ui.label_logo->setPixmap(image);

//    scene = new QGraphicsScene(this);
//    ui.graphicsView->setScene(scene);
//    QBrush greenBrush(Qt::green);
//    QBrush blueBrush(Qt::blue);
//    QPen outlinePen(Qt::black);
//    outlinePen.setWidth(2);

    //rectangle = scene->addRect(100, 0, 80, 100, outlinePen, blueBrush);
    // addEllipse(x,y,w,h,pen,brush)
    //ellipse = scene->addEllipse(0, -100, 300, 60, outlinePen, greenBrush);
    //text = scene->addText("bogotobogo.com", QFont("Arial", 20) );
    // movable text
    //text->setFlag(QGraphicsItem::ItemIsMovable);

    //Viewer* viewer = new Viewer();
    //ui.viewerLayout->addWidget(viewer);
    QObject::connect(&qnode, SIGNAL(imageUpdated(QImage*)), this, SLOT(cameraViewUpdate(QImage*)));

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
  close();
}

void MainWindow::cameraViewUpdate(QImage* img){
  QPixmap pix = QPixmap::fromImage(*img);

  scene = new QGraphicsScene(this);
  ui.graphicsView->setScene(scene);

  QBrush greenBrush(Qt::green);
  QBrush blueBrush(Qt::blue);
  QPen outlinePen(Qt::black);
  outlinePen.setWidth(2);

  scene->setBackgroundBrush(pix);
  delete img;

//  rectangle = scene->addRect(100, 0, 80, 100, outlinePen, blueBrush);
//  //addEllipse(x,y,w,h,pen,brush)
//  ellipse = scene->addEllipse(0, -100, 300, 60, outlinePen, greenBrush);
//  text = scene->addText("bogotobogo.com", QFont("Arial", 20) );
//  //movable text
//  text->setFlag(QGraphicsItem::ItemIsMovable);
}

void MainWindow::on_pushButton_raw_image_clicked(){
  scene = new QGraphicsScene(this);
  ui.graphicsView->setScene(scene);
  QBrush greenBrush(Qt::green);
  QBrush blueBrush(Qt::blue);
  QPen outlinePen(Qt::black);
  outlinePen.setWidth(2);

  rectangle = scene->addRect(100, 0, 80, 100, outlinePen, blueBrush);
  //addEllipse(x,y,w,h,pen,brush)
  ellipse = scene->addEllipse(0, -100, 300, 60, outlinePen, greenBrush);
  text = scene->addText("bogotobogo.com", QFont("Arial", 20) );
  //movable text
  text->setFlag(QGraphicsItem::ItemIsMovable);
}

void MainWindow::on_pushButton_depth_image_clicked(){

}
void MainWindow::on_pushButton_pcl_data_clicked(){

}


void MainWindow::on_button_connect_clicked(bool check ) {

  //ui.progressBar->setValue(50);
  //qnode.emit50();
  if ( !qnode.init() ) {
        showNoMasterMessage();
     } else {
        ui.button_connect->setEnabled(true);
           }
//  while (true)
//  {
//      sensor = qnode.sensor_data;
//      ui.progressBar_2->setValue(33);
//  }

}

void MainWindow::on_joystick_offset_data_update(float x, float y, float z ){
    ui.lineEdit_x->setText(QString::number(x, 'g',3));
    ui.lineEdit_y->setText(QString::number(y, 'g',3));
    ui.lineEdit_z->setText(QString::number(z, 'g',3));
}

void MainWindow::on_joystick_rot_offset_data_update(float rot_x, float rot_y, float rot_z ){
    ui.lineEdit_rot_x->setText(QString::number(rot_x, 'g',3));
    ui.lineEdit_rot_y->setText(QString::number(rot_y, 'g',3));
    ui.lineEdit_rot_z->setText(QString::number(rot_z, 'g',3));
}

void MainWindow::on_sensor_data_update(int tactile_data, int length_data, int ultrasonic_data){
    ui.progressBar->setValue(tactile_data);
    ui.progressBar_2->setValue(length_data);
    ui.progressBar_3->setValue(ultrasonic_data);
}

void MainWindow::on_robot_pos_update(double x, double y, double z, double rx, double ry, double rz){
    double robot_x = x;
    double robot_y = y;
    double robot_z = z;
    double robot_rx = rx;
    double robot_ry = ry;
    double robot_rz = rz;

    ROS_INFO("Robot end-effector pose have been updated!!!");
    ui.lineEdit_4->setText(QString::number(robot_x, 'g',6));
    ui.lineEdit_5->setText(QString::number(robot_y, 'g',6));
    ui.lineEdit_6->setText(QString::number(robot_z, 'g',6));
    ui.lineEdit_7->setText(QString::number(robot_rx, 'g',6));
    ui.lineEdit_8->setText(QString::number(robot_ry, 'g',6));
    ui.lineEdit_9->setText(QString::number(robot_rz, 'g',6));
}

void MainWindow::on_ft_sensor_update(double Fx, double Fy, double Fz, double Mx, double My, double Mz)
{
  ui.lineEdit_Fx->setText(QString::number(Fx, 'g',6));
  ui.lineEdit_Fy->setText(QString::number(Fy, 'g',6));
  ui.lineEdit_Fz->setText(QString::number(Fz, 'g',6));
  ui.lineEdit_Mx->setText(QString::number(Mx, 'g',6));
  ui.lineEdit_My->setText(QString::number(My, 'g',6));
  ui.lineEdit_Mz->setText(QString::number(Mz, 'g',6));
}

void MainWindow::on_robot_joint_update(double j1, double j2, double j3, double j4, double j5, double j6)
{
  ROS_INFO("Robot end-effector pose have been updated!!!");
  ui.lineEdit_j1->setText(QString::number(j1, 'g',6));
  ui.lineEdit_j2->setText(QString::number(j2, 'g',6));
  ui.lineEdit_j3->setText(QString::number(j3, 'g',6));
  ui.lineEdit_j4->setText(QString::number(j4, 'g',6));
  ui.lineEdit_j5->setText(QString::number(j5, 'g',6));
  ui.lineEdit_j6->setText(QString::number(j6, 'g',6));
}

void MainWindow::on_motor_calibration_clicked(){
  std::string param = "";
  param ="c,1";
  qnode.motorCmdPublisher(param);

}

void MainWindow::on_motor_forward_clicked(){
  std::string param = "";
  param ="q";
  qnode.motorCmdPublisher(param);
}

void MainWindow::on_motor_backward_clicked(){
  std::string param = "";
  param ="p";
  qnode.motorCmdPublisher(param);
}

void MainWindow::on_motor_control_clicked(){
  std::string param = "";
  int motor_circle = 0;
  std::stringstream ss;
  motor_circle = ui.spinBox->value();
  ss << "f," << motor_circle;
  param = ss.str();
  qnode.motorCmdPublisher(param);
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
//	bool enabled;
//	if ( state == 0 ) {
//		enabled = true;
//	} else {
//		enabled = false;
//	}
//	ui.line_edit_master->setEnabled(enabled);
//	ui.line_edit_host->setEnabled(enabled);
//	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
//    QSettings settings("Qt-Ros Package", "robotlinker_gui");
//    restoreGeometry(settings.value("geometry").toByteArray());
//    restoreState(settings.value("windowState").toByteArray());
//    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
//    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
//    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
//    ui.line_edit_master->setText(master_url);
//    ui.line_edit_host->setText(host_url);
//    //ui.line_edit_topic->setText(topic_name);
//    bool remember = settings.value("remember_settings", false).toBool();
//    ui.checkbox_remember_settings->setChecked(remember);
//    bool checked = settings.value("use_environment_variables", false).toBool();
//    ui.checkbox_use_environment->setChecked(checked);
//    if ( checked ) {
//    	ui.line_edit_master->setEnabled(false);
//    	ui.line_edit_host->setEnabled(false);
//    	//ui.line_edit_topic->setEnabled(false);
//    }
}

void MainWindow::WriteSettings() {
//    QSettings settings("Qt-Ros Package", "robotlinker_gui");
//    settings.setValue("master_url",ui.line_edit_master->text());
//    settings.setValue("host_url",ui.line_edit_host->text());
//    //settings.setValue("topic_name",ui.line_edit_topic->text());
//    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());
//    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    //WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace robotlinker_gui

