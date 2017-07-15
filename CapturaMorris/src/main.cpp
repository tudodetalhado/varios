#define _CRT_SECURE_NO_WARNINGS

#include "MainWindow.h"
#include <QApplication>
#include <QRect>
#include <QDesktopWidget>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow mainWindow;
    QRect screenGeometry = QApplication::desktop()->screenGeometry();
    int x = (screenGeometry.width()-mainWindow.width()) / 2;
    //int y = (screenGeometry.height()-mainWindow.height()) / 2;
    mainWindow.move(x, 0);
    mainWindow.show();
    return app.exec();
}
