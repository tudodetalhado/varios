#include "mainwindow.h"
#include <QApplication>
#include <QDesktopWidget>

int main(int argc, char *argv[])
{
    //QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    //QApplication a(argc, argv);

    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);
    MainWindow mainWindow;
    QRect rect = QApplication::desktop()->screenGeometry();
    //int x = (rect.width()-mainWindow.width()) / 2;
    //int y = (rect.height()-mainWindow.height()) / 2;

    int w = 1960; //rect.width()  * 1;
    int h = 990;//rect.height() * 1;
    //mainWindow.setFixedSize(w,h);

    mainWindow.setGeometry(0,40,w,h);


    //mainWindow.setFixedSize(screenGeometry.width(),screenGeometry.height());
    //mainWindow.move(x, 0);

    //720 - 20
    mainWindow.show();

    return app.exec();
}
