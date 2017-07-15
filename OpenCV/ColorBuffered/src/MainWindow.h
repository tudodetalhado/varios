#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "stdafx.h"
#include <Kinect.h>
#include <QMainWindow>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <thread>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_btnCapturar_clicked();

    void on_btnParar_clicked();

private:
    Ui::MainWindow *ui;
    IKinectSensor* pSensor;
    IColorFrameReader* pColorReader;
    cv::Mat colorMat;
    void inicializarSensor();
    void capturarCor();

    int width;
    int height;
    bool _deveCapturar;
};

#endif // MAINWINDOW_H
