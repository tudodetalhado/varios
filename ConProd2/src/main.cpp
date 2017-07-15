#include "conproddialog.h"
#include <QApplication>
#include "conproddialog.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ConProdDialog w;
    w.setWindowTitle("Semaphore: Consumer & Producer");
    w.show();

    return a.exec();
}
