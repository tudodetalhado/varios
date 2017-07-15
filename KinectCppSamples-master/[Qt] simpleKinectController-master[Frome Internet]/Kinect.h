#ifndef KINECT_H
#define KINECT_H

#include <Windows.h>
#include <NuiApi.h>
#include <QMap>
#include <QObject>

class Kinect : public QObject
{
    Q_OBJECT
public:
    Kinect(INuiSensor* nui);
    ~Kinect();

    INuiSensor* getSensor(){return nui;}
    HRESULT initialize();
    HRESULT uninitialize();
private:
    INuiSensor* nui;
};



#endif // KINECT_H
