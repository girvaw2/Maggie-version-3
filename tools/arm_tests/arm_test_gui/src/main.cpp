#include <QtGui/QApplication>
#include "widget.h"
//#include <ros/ros.h>

int main(int argc, char *argv[])
{
//    int x;
//    ros::init (x, (char **)0, "maggie_move_arm_test");

    QApplication a(argc, argv);
    Widget w;
    w.show();

    return a.exec();
}
