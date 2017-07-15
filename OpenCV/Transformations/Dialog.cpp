#include "Dialog.h"
#include "ui_Dialog.h"

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::paintEvent(QPaintEvent *e)
{
    QPainter painter(this);

    QPen pen1(Qt::black);
    pen1.setWidth(6);

    QPen pen2(Qt::red);
    pen2.setWidth(6);

    QPen pen3(Qt::green);
    pen3.setWidth(6);

    QPen pen4(Qt::blue);
    pen4.setWidth(6);

    QRect rect(100,100,100,100);

    painter.setPen(pen1);
    painter.drawRect(rect);
}
