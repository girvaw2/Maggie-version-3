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
    tb = new TrackBall( ui->hueLowerSlider->value(),
                        ui->hueUpperSlider->value(),
                        ui->saturationLowerSlider->value(),
                        ui->saturationUpperSlider->value(),
                        ui->valueLowerSlider->value(),
                        ui->valueUpperSlider->value());

    tb_thread = new boost::thread (boost::bind(&TrackBall::loop, boost::ref(*tb)));
}

void Widget::on_hueLowerSlider_sliderMoved(int position)
{
    boost::signals2::signal<void(int)> signal;
    signal.connect(boost::bind(&TrackBall::setHueLowerValue, boost::ref(*tb), _1));
    signal(position);
}

void Widget::on_hueUpperSlider_sliderMoved(int position)
{
    boost::signals2::signal<void(int)> signal;
    signal.connect(boost::bind(&TrackBall::setHueUpperValue, boost::ref(*tb), _1));
    signal(position);
}
