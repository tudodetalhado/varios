#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

private slots:
    void Read();
    void SliderFramePress();
    void SliderFrameRelease();
    void SliderFrameMove(int);
    void on_pushButton_clicked();
    void on_OpenFile_clicked();
    void on_Play_clicked();
    void on_Pause_clicked();
    void on_Stop_clicked();
};

#endif // MAINWINDOW_H
