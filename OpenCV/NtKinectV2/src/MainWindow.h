#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <chrono>
#include <thread>
#include "NtKinect.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public slots:
    void updateKinectData();

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_btnGravar_clicked();
    void on_btnParar_clicked();

private:
    Ui::MainWindow *ui;
    bool _deveGravar = false;
    cv::VideoWriter vw;
};

#endif // MAINWINDOW_H
