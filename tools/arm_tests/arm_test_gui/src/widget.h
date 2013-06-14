#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "ui_widget.h"
#include "arm_test_gui/trackball.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/signals2.hpp>

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

    void on_hueLowerSlider_sliderMoved(int position);

    void on_hueUpperSlider_sliderMoved(int position);

    void on_saturationLowerSlider_sliderMoved(int position);

    void on_saturationUpperSlider_sliderMoved(int position);

    void on_valueLowerSlider_sliderMoved(int position);

    void on_valueUpperSlider_sliderMoved(int position);

    void on_tabWidget_selected(const QString &arg1);

    void on_headTrackingPushButton_clicked();

    void on_headBallTrackerCheckBox_toggled(bool checked);

private:
    Ui::Widget *ui;
    boost::shared_ptr<boost::thread> face_track_ptr;
    boost::thread *tb_thread;
    boost::shared_ptr<TrackBall> track_ball_ptr;
};

#endif // WIDGET_H
