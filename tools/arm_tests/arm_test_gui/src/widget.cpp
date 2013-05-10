#include "ui_widget.h"
#include "widget.h"
#include "arm_test_gui/parkarm.h"
#include "arm_test_gui/seedarm.h"


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
