#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(atualizarDisplay()));
}

MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::atualizarDisplay()
{
    tempo += 1000;
    ui->lcdNumber->display(QString::number(tempo/1000));
}

void MainWindow::on_btnStart_clicked()
{
    if (!timer->isActive()){
        timer->start(1000);
    }
}

void MainWindow::on_btnStop_clicked()
{
   timer->stop();
}

void MainWindow::on_btnReiniciar_clicked()
{
    if (timer->isActive()){
        QMessageBox::information(
               this,
               tr("QTimerTest"),
               tr("timer is Active.") );
    }
}
