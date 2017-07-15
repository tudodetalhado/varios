#include <QtGui/QApplication>
#include "mainwindow.h"

extern "C"{
    #include "wrapper.h"
}

int main(int argc, char *argv[])
{
    service(); //calling the function service inside the wrapper

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
