#include "Kinect.h"

Kinect::Kinect(IKinectSensor *pKinectSensor)
{
    this->m_pKinectSensor = pKinectSensor;
}

Kinect::~Kinect(){

}

HRESULT Kinect::initialize(){
 return S_OK;
}

HRESULT Kinect::uninitialize(){
    return S_OK;
}
