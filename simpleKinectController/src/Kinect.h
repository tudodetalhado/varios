#ifndef KINECT_H
#define KINECT_H

#include <Windows.h>
#include <kinect.h>
#include <QMap>
#include <QObject>

class Kinect : public QObject
{
    Q_OBJECT
public:
    Kinect(IKinectSensor* pKinectSensor);
    ~Kinect();

    IKinectSensor* getSensor(){return nui;}
    HRESULT initialize();
    HRESULT uninitialize();
private:
    IKinectSensor* m_pKinectSensor;
};



#endif // KINECT_H
