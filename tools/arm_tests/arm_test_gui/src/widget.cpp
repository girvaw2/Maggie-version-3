#include "ui_widget.h"
#include "widget.h"


#include <iostream>

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



void Widget::on_pushButton_clicked()
{
    std::cout << "Button pushed" << std::endl;
}

void Widget::on_pushButton_pressed()
{
    std::cout << "Button pressed" << std::endl;

}
