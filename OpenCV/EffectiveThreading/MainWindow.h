#ifndef __MAIN_WINDOW_H__
#define __MAIN_WINDOW_H__

#include <QMainWindow>

#include "ui_mainwindow.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        MainWindow(QWidget *parent=0);

    private slots:
        void updateCount(int cnt);
        void updateInfiniteCount(int cnt);

        void startCount();
        void startInfiniteCount();

        void countFinished();
        void infiniteCountFinished();

    private:
        void connectSignalsSlots();

        bool countRunning;
        bool infiniteCountRunning;

        Ui_MainWindow ui;
};

#endif // __MAIN_WINDOW_H__
