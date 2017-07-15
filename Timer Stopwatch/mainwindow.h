#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QSoundEffect>

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    QTimer *ttimer = new QTimer(this), *stimer = new QTimer(this);
    unsigned stime = 0;
    unsigned previousTime = 0;
    unsigned short slaps = 0;
    int ttime = 0;
    QSoundEffect *sound = new QSoundEffect(this);
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_sstart_pressed();
    void on_stimer();
    void on_spause_pressed();
    void on_slap_pressed();
    void on_sreset_pressed();

    void on_tstart_pressed();
    void on_ttimer();
    void on_tpause_pressed();
    void on_tcancel_pressed();
    void on_treset_pressed();
    void alarm();

    void refresh_display();
    void on_hr_valueChanged(int);
    void on_min_valueChanged(int);
    void on_sec_valueChanged(int);
    void on_milli_valueChanged(int);

signals:
    void timeUp();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
