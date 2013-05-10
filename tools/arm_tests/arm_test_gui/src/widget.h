#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "ui_widget.h"

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

private:
    Ui::Widget *ui;
};

#endif // WIDGET_H
