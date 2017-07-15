#ifndef KINECTMANAGER_H
#define KINECTMANAGER_H
#include "Kinect.h"

class KinectManager : public QObject
{
    Q_OBJECT
public:
    KinectManager();
    ~KinectManager();
    HRESULT initialize();

private:
    HRESULT fillMaps();
    QMap<int,QString> nameMap;
    QMap<int, INuiSensor*> kinectMap;
    Kinect* kinect;
    static void CALLBACK KinectManager::OnSensorStatusChanged( HRESULT hr, const OLECHAR* instanceName, const OLECHAR* uniqueDeviceName, void* userData);

signals:
    void mapChanged(QMap<int,QString> kinectMap);
    void changeSelection(int i);
    void error (QString error);
};
#endif // KINECTMANAGER_H
