#include "DepthBasics.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CDepthBasics w;
    w.show();

    return a.exec();
}
