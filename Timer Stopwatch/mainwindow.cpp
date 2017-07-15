#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ttimer->setTimerType(Qt::PreciseTimer);
    stimer->setTimerType(Qt::PreciseTimer);
    connect(ttimer, SIGNAL(timeout()), this, SLOT(on_ttimer()));
    connect(this, SIGNAL(timeUp()), this, SLOT(alarm()));
    sound->setSource(QUrl("qrc:/alarm/alarm.wav"));
    sound->setVolume(1.0f);
    connect(stimer, SIGNAL(timeout()), this, SLOT(on_stimer()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_sstart_pressed()
{
    stimer->start(10);
}

void MainWindow::on_stimer()
{
    if (!stimer->isActive()) return;
    ++stime;
    const int hr = stime / 360000, min = stime % 360000 / 6000, sec = stime % 6000 / 100, milli = stime % 100;
    ui->smilli2->display(milli % 10);
    ui->smilli1->display(milli / 10);
    ui->ssec2->display(sec % 10);
    ui->ssec1->display(sec / 10);
    ui->smin2->display(min % 10);
    ui->smin1->display(min / 10);
    ui->shr2->display(hr % 10);
    ui->shr1->display(hr / 10);
    if (stime >= 8640000)
    {
        stimer->stop();
        stime = 0;
    }
}

void MainWindow::on_spause_pressed()
{
    stimer->stop();
}

void MainWindow::on_slap_pressed()
{
    ++slaps;
    const unsigned hr1 = ui->shr1->value(), hr2 = ui->shr2->value(),
                   min1 = ui->smin1->value(), min2 = ui->smin2->value(),
                   sec1 = ui->ssec1->value(), sec2 = ui->ssec2->value(),
                   milli1 = ui->smilli1->value(), milli2 = ui->smilli2->value();
    const unsigned currentTime = stime, elapsedTime = currentTime - previousTime,
                   ehr1 = elapsedTime / 360000 / 10, ehr2 = elapsedTime / 360000 % 10,
                   emin1 = elapsedTime % 360000 / 6000 / 10, emin2 = elapsedTime % 360000 / 6000 % 10,
                   esec1 = elapsedTime % 6000 / 100 / 10, esec2 = elapsedTime % 6000 / 100 % 10,
                   emilli1 = elapsedTime % 100 / 10, emilli2 = elapsedTime % 100 % 10;

    ui->laps->appendPlainText
    (
        QString::number(slaps) + "                " + QString::number(hr1) + QString::number(hr2) + ":" +
        QString::number(min1) + QString::number(min2) +  ":" + QString::number(sec1) +
        QString::number(sec2) + ":" + QString::number(milli1) + QString::number(milli2) + "                " +
        QString::number(ehr1) + QString::number(ehr2) + ":" + QString::number(emin1) + QString::number(emin2) + ":" +
        QString::number(esec1) + QString::number(esec2) + ":" + QString::number(emilli1) + QString::number(emilli2)
    );
    previousTime = currentTime;
}

void MainWindow::on_sreset_pressed()
{
    stimer->stop();
    stime = 0;
    slaps = 0;
    previousTime = 0;
    ui->laps->clear();
    ui->smilli2->display(0);
    ui->smilli1->display(0);
    ui->ssec2->display(0);
    ui->ssec1->display(0);
    ui->smin2->display(0);
    ui->smin1->display(0);
    ui->shr2->display(0);
    ui->shr1->display(0);
}

void MainWindow::on_tstart_pressed()
{
    sound->stop();
    if (ttime == 0)
    {
        refresh_display();
        ttime = ui->hr->value() * 360000 + ui->min->value() * 6000 + ui->sec->value() * 100 + ui->milli->value();
    }
    if (ttime != 0)
    {
        ttimer->start(10);
        ui->hr->setDisabled(true);
        ui->min->setDisabled(true);
        ui->sec->setDisabled(true);
        ui->milli->setDisabled(true);
    }
}

void MainWindow::on_ttimer()
{
    if (!ttimer->isActive()) return;
    --ttime;
    const int hr = ttime / 360000, min = ttime % 360000 / 6000, sec = ttime % 6000 / 100, milli = ttime % 100;
    ui->tmilli2->display(milli % 10);
    ui->tmilli1->display(milli / 10);
    ui->tsec2->display(sec % 10);
    ui->tsec1->display(sec / 10);
    ui->tmin2->display(min % 10);
    ui->tmin1->display(min / 10);
    ui->thr2->display(hr % 10);
    ui->thr1->display(hr / 10);
    if (ttime <= 0)
    {
        emit timeUp();
        ttimer->stop();
        ui->hr->setEnabled(true);
        ui->min->setEnabled(true);
        ui->sec->setEnabled(true);
        ui->milli->setEnabled(true);
    }
}

void MainWindow::on_tpause_pressed()
{
    ttimer->stop();
    sound->stop();
}

void MainWindow::on_tcancel_pressed()
{
    ui->hr->setEnabled(true);
    ui->min->setEnabled(true);
    ui->sec->setEnabled(true);
    ui->milli->setEnabled(true);
    ttimer->stop();
    sound->stop();
    ttime = 0;
    refresh_display();
}

void MainWindow::on_treset_pressed()
{
    on_tcancel_pressed();
    ui->hr->setValue(0);
    ui->min->setValue(0);
    ui->sec->setValue(0);
    ui->milli->setValue(0);
}

void MainWindow::alarm()
{
    sound->play();
    setWindowState(windowState() ^ Qt::WindowMinimized);
    setWindowState((windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
    activateWindow();
    ui->mode->setCurrentIndex(0);
}

void MainWindow::refresh_display()
{
    ui->tmilli2->display(ui->milli->value() % 10);
    ui->tmilli1->display(ui->milli->value() / 10);
    ui->tsec2->display(ui->sec->value() % 10);
    ui->tsec1->display(ui->sec->value() / 10);
    ui->tmin2->display(ui->min->value() % 10);
    ui->tmin1->display(ui->min->value() / 10);
    ui->thr2->display(ui->hr->value() % 10);
    ui->thr1->display(ui->hr->value() / 10);
}

void MainWindow::on_hr_valueChanged(int arg1)
{
    ui->thr2->display(arg1 % 10);
    ui->thr1->display(arg1 / 10);
}

void MainWindow::on_min_valueChanged(int arg1)
{
    ui->tmin2->display(arg1 % 10);
    ui->tmin1->display(arg1 / 10);
}

void MainWindow::on_sec_valueChanged(int arg1)
{
    ui->tsec2->display(arg1 % 10);
    ui->tsec1->display(arg1 / 10);
}

void MainWindow::on_milli_valueChanged(int arg1)
{
    ui->tmilli2->display(arg1 % 10);
    ui->tmilli1->display(arg1 / 10);
}
