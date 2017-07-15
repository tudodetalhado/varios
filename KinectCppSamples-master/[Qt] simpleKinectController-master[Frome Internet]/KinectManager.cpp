#include "KinectManager.h"

KinectManager::KinectManager()
{
    kinect = NULL;
}

KinectManager::~KinectManager(){
    if (kinect !=  NULL){
        kinect->uninitialize();
    }
}

HRESULT KinectManager::initialize(){
    HRESULT hr = fillMaps();
    if (FAILED(hr)) return hr;
    NuiSetDeviceStatusCallback(OnSensorStatusChanged, this);
    return S_OK;
}

HRESULT KinectManager::fillMaps(){
    kinectMap.clear();
    nameMap.clear();
    HRESULT hr;
    int kinectCount;
    INuiSensor * nui;
    hr = NuiGetSensorCount(&kinectCount);
    if (FAILED(hr))return hr;
    for (int i = 0; i < kinectCount; i++)
    {
        QString text = "Kinect" + QString::number(i) + " ";
        hr = NuiCreateSensorByIndex(i, &nui);
        switch(hr)
        {
        case E_OUTOFMEMORY:
            text += "<Could not allocate memory>";
            continue;
        case E_POINTER:
            text += "<Unknown error (E_POINTER)>";
            continue;
        default:
            break;
        }
        hr = nui->NuiStatus();

        switch(hr)
        {
        case S_OK:
            kinectMap.insert(i,nui);
            break;
        case S_NUI_INITIALIZING:
            text += "<Still initializing>";
            break;
        case E_NUI_NOTCONNECTED:
            text += "<Kinect not connected>";
            break;
        case E_NUI_NOTGENUINE:
            text += "<Device is not a valid Kinect>";
            break;
        case E_NUI_NOTSUPPORTED:
            text +="<The Kinect is not a supported Kinect>";
            break;
        case E_NUI_INSUFFICIENTBANDWIDTH:
            text += "<Not enough USB bandwidth available>";
            break;
        case E_NUI_NOTPOWERED:
            text += "<The Kinect is not powered>";
            break;
        case E_NUI_NOTREADY:
            text += "<Unspecified error (E_NUI_NOT_READY)";
            break;
        default:
            break;
        }
        nameMap.insert(i,text);
        //nui->Release();
    }
    emit mapChanged(nameMap);
    if (kinectMap.isEmpty()){
        emit error("No usable Kinects found");
    }
    else if (kinect == NULL){
        kinect = new Kinect(kinectMap.begin().value());
        kinect->initialize();
        emit changeSelection(kinectMap.begin().key());
        emit error("");
    }
    return S_OK;
}

void CALLBACK KinectManager::OnSensorStatusChanged( HRESULT hr, const OLECHAR* instanceName, const OLECHAR* uniqueDeviceName, void* userData)
{
    KinectManager* pThis = (KinectManager*)userData;
    pThis->fillMaps();
}
