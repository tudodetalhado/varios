#include "MainWindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    //w.setFixedSize(1366, 750);
    w.show();

    return a.exec();
}
