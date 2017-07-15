#include <QMessageBox>
#include <QThread>

#include "mainwindow.h"
#include "countworker.h"
#include "infinitecountworker.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      countRunning(false),
      infiniteCountRunning(false)
{
    ui.setupUi(this);
    connectSignalsSlots();
}

void MainWindow::updateCount(int cnt)
{
    ui.countOut->setText(QString::number(cnt));
}

void MainWindow::updateInfiniteCount(int cnt)
{
    ui.infiniteCountOut->setText(QString::number(cnt));
}

void MainWindow::startCount()
{
    QThread     *workerThread;
    CountWorker *worker;

    if (countRunning) {
        QMessageBox::critical(this, "Error", "Count is already running!");
        return;
    }

    workerThread = new QThread;
    worker       = new CountWorker(ui.countStart->value(), ui.countEnd->value());
    worker->moveToThread(workerThread);
    connect(workerThread, SIGNAL(started()), worker, SLOT(doWork()));
    connect(worker, SIGNAL(finished()), workerThread, SLOT(quit()));
    connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
    connect(worker, SIGNAL(finished()), this, SLOT(countFinished()));
    connect(workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()));
    connect(worker, SIGNAL(updateCount(int)), this, SLOT(updateCount(int)));
    workerThread->start();

    countRunning = true;
}

void MainWindow::startInfiniteCount()
{
    QThread             *workerThread;
    KinectTrack *worker;

    if (infiniteCountRunning) {
        QMessageBox::critical(this, "Error", "Infinite count is already running!");
        return;
    }

    workerThread = new QThread;
    worker       = new KinectTrack;
    worker->moveToThread(workerThread);
    connect(workerThread, SIGNAL(started()), worker, SLOT(doWork()));
    connect(worker, SIGNAL(finished()), workerThread, SLOT(quit()));
    connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
    connect(workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()));
    connect(worker, SIGNAL(finished()), this, SLOT(infiniteCountFinished()));
    connect(worker, SIGNAL(updateCount(int)), this, SLOT(updateInfiniteCount(int)));
    connect(ui.infiniteCountNoGo, SIGNAL(clicked()), worker, SLOT(stopWork()));
    workerThread->start();

    infiniteCountRunning = true;
}

void MainWindow::countFinished()
{
    countRunning = false;
}

void MainWindow::infiniteCountFinished()
{
    infiniteCountRunning = false;
}

void MainWindow::connectSignalsSlots()
{
    connect(ui.countGo, SIGNAL(clicked()), this, SLOT(startCount()));
    connect(ui.infiniteCountGo, SIGNAL(clicked()), this, SLOT(startInfiniteCount()));
}
