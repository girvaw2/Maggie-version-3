#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "ui_widget.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace Ui {
    class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();

private Q_SLOTS:

    void on_parkArmPushButton_clicked();

    void on_seedArmPushButton_clicked();

    void on_moveToIKPushButton_clicked();

    void on_startFaceTrackingPushButton_clicked();

    void on_stopFaceTrackingPushButton_clicked();

    void on_startBallTrackingPushButton_clicked();

private:
    Ui::Widget *ui;
    boost::shared_ptr<boost::thread> face_track_ptr;
};

#endif // WIDGET_H
