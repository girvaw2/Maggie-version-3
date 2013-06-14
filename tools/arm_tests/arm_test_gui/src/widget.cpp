#include "ui_widget.h"
#include "widget.h"
#include "arm_test_gui/parkarm.h"
#include "arm_test_gui/seedarm.h"
#include "arm_test_gui/ikhelper.h"
#include "arm_test_gui/trackface.h"

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_parkArmPushButton_clicked()
{
    ParkArm pa;
    pa.park();
}

void Widget::on_seedArmPushButton_clicked()
{
    SeedArm sa;
    sa.seed();
}

void Widget::on_moveToIKPushButton_clicked()
{
    IKHelper ik;

    double start_orientation[] = {0.655082, -0.393060, 0.095405, 0.638176};

    geometry_msgs::Pose pose;

    pose.position.x = ui->xDoubleSpinBox->value();
    pose.position.y = ui->yDoubleSpinBox->value();
    pose.position.z = ui->zDoubleSpinBox->value();

    pose.orientation.x = start_orientation[0];
    pose.orientation.y = start_orientation[1];
    pose.orientation.z = start_orientation[2];
    pose.orientation.w = start_orientation[3];

    ik.moveToGoal(pose);
}

void Widget::on_startFaceTrackingPushButton_clicked()
{

    TrackFace tf;
    face_track_ptr.reset(new boost::thread (boost::bind(&TrackFace::startFaceTracking, &tf)));
    //tf.startFaceTracking();
    //face_track_ptr.reset(new boost::thread (boost::bind(&TrackFace::moveArmToFacePosition, &tf)));

}

void Widget::on_stopFaceTrackingPushButton_clicked()
{
    //face_track_ptr->interrupt();
    //face_track_ptr->detach();
    face_track_ptr->join();
}

void Widget::on_startBallTrackingPushButton_clicked()
{
    track_ball_ptr.reset(new TrackBall( ui->hueLowerSlider->value(),
                        ui->hueUpperSlider->value(),
                        ui->saturationLowerSlider->value(),
                        ui->saturationUpperSlider->value(),
                        ui->valueLowerSlider->value(),
                        ui->valueUpperSlider->value()));

    tb_thread = new boost::thread (boost::bind(&TrackBall::loop, boost::ref(track_ball_ptr)));
}

void Widget::on_hueLowerSlider_sliderMoved(int position)
{
    ui->hueLowerLabel->setText(QString::number(position));
    boost::signals2::signal<void(int)> signal;
    signal.connect(boost::bind(&TrackBall::setHueLowerValue, boost::ref(track_ball_ptr), _1));
    signal(position);
}

void Widget::on_hueUpperSlider_sliderMoved(int position)
{
    ui->hueUpperLabel->setText(QString::number(position));
    boost::signals2::signal<void(int)> signal;
    signal.connect(boost::bind(&TrackBall::setHueUpperValue, boost::ref(track_ball_ptr), _1));
    signal(position);
}

void Widget::on_saturationLowerSlider_sliderMoved(int position)
{
    ui->saturationLowerLabel->setText(QString::number(position));
    boost::signals2::signal<void(int)> signal;
    signal.connect(boost::bind(&TrackBall::setSaturationLowerValue, boost::ref(track_ball_ptr), _1));
    signal(position);
}

void Widget::on_saturationUpperSlider_sliderMoved(int position)
{
    ui->saturationUpperLabel->setText(QString::number(position));
    boost::signals2::signal<void(int)> signal;
    signal.connect(boost::bind(&TrackBall::setSaturationUpperValue, boost::ref(track_ball_ptr), _1));
    signal(position);
}

void Widget::on_valueLowerSlider_sliderMoved(int position)
{
    ui->valueLowerLabel->setText(QString::number(position));
    boost::signals2::signal<void(int)> signal;
    signal.connect(boost::bind(&TrackBall::setValueLowerValue, boost::ref(track_ball_ptr), _1));
    signal(position);
}

void Widget::on_valueUpperSlider_sliderMoved(int position)
{
    ui->valueUpperLabel->setText(QString::number(position));
    boost::signals2::signal<void(int)> signal;
    signal.connect(boost::bind(&TrackBall::setValueUpperValue, boost::ref(track_ball_ptr), _1));
    signal(position);
}

void Widget::on_tabWidget_selected(const QString &arg1)
{
    if (arg1.compare("Ball Tracking") == 0)
    {
        ui->hueLowerLabel->setText(QString::number(ui->hueLowerSlider->value()));
        ui->hueUpperLabel->setText(QString::number(ui->hueUpperSlider->value()));
        ui->saturationLowerLabel->setText(QString::number(ui->saturationLowerSlider->value()));
        ui->saturationUpperLabel->setText(QString::number(ui->saturationUpperSlider->value()));
        ui->valueLowerLabel->setText(QString::number(ui->valueLowerSlider->value()));
        ui->valueUpperLabel->setText(QString::number(ui->valueUpperSlider->value()));
    }
}

void Widget::on_headTrackingPushButton_clicked()
{
}

void Widget::on_headBallTrackerCheckBox_toggled(bool checked)
{
    boost::signals2::signal<void(bool)> signal;
    signal.connect(boost::bind(&TrackBall::headBallTrack, boost::ref(track_ball_ptr), _1));
    signal(checked);
}
