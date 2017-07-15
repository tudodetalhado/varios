#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <chrono>
#include <thread>
#include <QThread>
#include <QMainWindow>
#include "KinectSensor.h"
#include "KinectTime.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    static const int cColorWidth  = 1920;
    static const int cColorHeight = 1080;

private slots:
    void onKinectCallback(RGBQUAD *buffer, KinectTime *time);
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    KinectSensor *sensor;
    void FPS(QLabel *label, INT64 time);
    INT64 timeInicial = 0;
    INT64 ultimaContagem;
    INT64 proximaAtualizacaoTime;
    DWORD totalDeframesAtualizados;
    double frequencia;
};

#endif // MAINWINDOW_H
