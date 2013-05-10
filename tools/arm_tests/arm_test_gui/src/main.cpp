#include <QtGui/QApplication>
#include "widget.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "arm_test_gui");

    QApplication a(argc, argv);
    Widget w;
    w.show();

    int result = a.exec();

    ros::shutdown();

    return result;
}
