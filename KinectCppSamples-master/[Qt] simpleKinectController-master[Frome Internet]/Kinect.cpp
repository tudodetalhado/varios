#include "Kinect.h"

Kinect::Kinect(INuiSensor *nui)
{
    this->nui = nui;
}

Kinect::~Kinect(){

}

HRESULT Kinect::initialize(){
 return S_OK;
}

HRESULT Kinect::uninitialize(){
    return S_OK;
}
