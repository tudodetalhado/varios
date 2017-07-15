#include "MainWindow.h"
#include <QApplication>
#include "KinectManager.h"
#include <QMap>

int main(int argc, char *argv[])
{
    //initialize GUI
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    KinectManager manager;
    typedef QMap<int,QString> KinectStringMap;
    qRegisterMetaType<KinectStringMap>("KinectStringMap");
    QObject::connect(&manager,SIGNAL( mapChanged(KinectStringMap)),&w,SLOT(setDropDownList(KinectStringMap)));
    QObject::connect(&manager,SIGNAL(changeSelection(int)),&w,SLOT(setComboBox(int)));
    QObject::connect(&manager,SIGNAL(error(QString)),&w,SLOT(displayError(QString)));
    HRESULT hr = manager.initialize();
    if (FAILED(hr)) w.displayError("Something big happend: " + QString::number(hr));
    //if (kinectList.size() == 0) w.displayError("No Kinects to be found.");

    return a.exec();
}


