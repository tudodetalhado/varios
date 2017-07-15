#include "KinectSensor.h"

KinectSensor::KinectSensor(QObject *parent, int milliseconds) :
    QThread(parent)
{
    this->milliseconds = milliseconds;
    kinectSensor = NULL;
    pCoordinateMapper = NULL;
    msFrameReader = NULL;
    bufferDeProfundidade = NULL;

    bufferDeCor = new RGBQUAD[colorFrameWidth * colorFrameHeight * sizeof(RGBQUAD)];

    _frmColor = cv::Mat::zeros(1080, 1920, CV_8UC4);

    _1920x1080_cor_buffer = new RGBQUAD[1920 * 1080];


    _time = new KinectTime();

    _colorBufferSize = 1920 * 1080 * sizeof(RGBQUAD);


    inicializar();
}

KinectSensor::~KinectSensor()
{

    SafeRelease(kinectSensor);
    SafeRelease(pCoordinateMapper);
    SafeRelease(msFrameReader);
}

bool KinectSensor::inicializar()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&kinectSensor);

    if (FAILED(hr))
    {
        inicializado = false;
        return inicializado;
    }

    if (kinectSensor)
    {
        kinectSensor->get_CoordinateMapper(&pCoordinateMapper);
        hr = kinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            kinectSensor->OpenMultiSourceFrameReader(
                FrameSourceTypes::FrameSourceTypes_Color |
                FrameSourceTypes::FrameSourceTypes_Depth |
                FrameSourceTypes::FrameSourceTypes_Infrared,
                &msFrameReader);
        }
    }

    inicializado = SUCCEEDED(hr);

    return inicializado;
}


void KinectSensor::run()
{
    KinectTime *time = new KinectTime();
    while(_deveCapturar)
    {
        if (capturarFrame())
        {
         time->valor = _colorTime;
         emit kinectCallback(_1920x1080_cor_buffer, time);
        }
         std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }
}

bool KinectSensor::capturarFrame()
{
    if (!inicializado)
    {
        return false;
    }

    bool capturou = false;

    IMultiSourceFrame *multiSrcFrm = NULL;
    HRESULT hr = msFrameReader->AcquireLatestFrame(&multiSrcFrm);

    if (SUCCEEDED(hr))
    {
        IColorFrameReference* colorFrmRef = NULL;
        hr = multiSrcFrm->get_ColorFrameReference(&colorFrmRef);

        if (SUCCEEDED(hr))
        {
            IColorFrame* colorFrm = NULL;
            hr = colorFrmRef->AcquireFrame(&colorFrm);

            if (SUCCEEDED(hr))
            {
                UINT colorBufferSize = 1920 * 1080 * sizeof(RGBQUAD);
                //UINT uBufferSize = 1080 * 1920 * 4 * sizeof(BYTE);

                ColorImageFormat imageFormat = ColorImageFormat_None;
                hr = colorFrm->get_RelativeTime(&_colorTime);
                hr = colorFrm->get_RawColorImageFormat(&imageFormat);

                //colorFrm->CopyConvertedFrameDataToArray(uBufferSize, frameColor.data, ColorImageFormat_Bgra);
                //colorFrm->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()),
                //        &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra);

                if (imageFormat == ColorImageFormat_Bgra)
                {
                    hr = colorFrm->AccessRawUnderlyingBuffer(&colorBufferSize, reinterpret_cast<BYTE**>(&bufferDeCor));
                }
                else
                {
                    hr = colorFrm->CopyConvertedFrameDataToArray(colorBufferSize,
                             reinterpret_cast<BYTE*>(_1920x1080_cor_buffer), ColorImageFormat_Bgra);
                }
                capturou = true;
                //processarCor();
            }
            SafeRelease(colorFrm);
        }
        SafeRelease(colorFrmRef);
    }
    SafeRelease(multiSrcFrm);

    return capturou;
}



RGBQUAD *KinectSensor::getColorFrame(IMultiSourceFrame* multiFrame)
{
    //Multi frame
    //IMultiSourceFrame* multiFrame = NULL;
    HRESULT hr = msFrameReader->AcquireLatestFrame(&multiFrame);

    if (!SUCCEEDED(hr))
    {
        return NULL;
    }

    IColorFrameReference* colorFrameReference = NULL;
    IColorFrame* colorFrame = NULL;
    multiFrame->get_ColorFrameReference(&colorFrameReference);
    hr = colorFrameReference->AcquireFrame(&colorFrame);

    if (SUCCEEDED(hr))
    {
        if (bufferDeCor == NULL)
        {
            IFrameDescription* pFrameDescription = NULL;
            hr = colorFrame->get_FrameDescription(&pFrameDescription);
            hr = pFrameDescription->get_Width(&colorFrameWidth);
            hr = pFrameDescription->get_Height(&colorFrameHeight);
            bufferDeCor = new RGBQUAD[colorFrameWidth * colorFrameHeight];
            SafeRelease(pFrameDescription);
        }

        UINT nBufferSize = colorFrameWidth * colorFrameHeight * sizeof(RGBQUAD);
        hr = colorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(bufferDeCor), ColorImageFormat_Bgra);
    }

    SafeRelease(multiFrame);
    SafeRelease(colorFrame);
    SafeRelease(colorFrameReference);

    return bufferDeCor;
}

UINT16* KinectSensor::getDepthFrame()
{
    //Multi frame
    IMultiSourceFrame* multiFrame = NULL;
    HRESULT hr = msFrameReader->AcquireLatestFrame(&multiFrame);

    if (!SUCCEEDED(hr))
    {
        return NULL;
    }

    IDepthFrameReference* depthFrameReference = NULL;
    IDepthFrame* depthFrame = NULL;
    multiFrame->get_DepthFrameReference(&depthFrameReference);
    hr = depthFrameReference->AcquireFrame(&depthFrame);

    if (SUCCEEDED(hr))
    {
        if (bufferDeProfundidade == NULL)
        {
            IFrameDescription* frameDescription = NULL;
            hr = depthFrame->get_FrameDescription(&frameDescription);
            frameDescription->get_Width(&depthFrameWidth);
            frameDescription->get_Height(&depthFrameHeight);
            bufferDeProfundidade = new UINT16[depthFrameHeight * depthFrameWidth];
            depthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
            depthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
            SafeRelease(frameDescription);
        }

        UINT bufferSize = depthFrameHeight * depthFrameWidth;
        //hr = depthFrame->CopyFrameDataToArray(bufferSize, bufferDeProfundidade);
        hr = depthFrame->AccessUnderlyingBuffer(&bufferSize, &bufferDeProfundidade);
    }

    SafeRelease(multiFrame);
    SafeRelease(depthFrame);
    SafeRelease(depthFrameReference);

    return bufferDeProfundidade;
}

//UINT16* KinectSensor::getDepthFrame()
//{
//    //Multi frame
//    IMultiSourceFrame* multiFrame = NULL;
//    HRESULT hr = multiSourceFrameReader->AcquireLatestFrame(&multiFrame);

//    if (!SUCCEEDED(hr))
//    {
//        return NULL;
//    }

//    IDepthFrameReference* depthFrameReference = NULL;
//    IDepthFrame* depthFrame = NULL;
//    multiFrame->get_DepthFrameReference(&depthFrameReference);
//    hr = depthFrameReference->AcquireFrame(&depthFrame);

//    if (SUCCEEDED(hr))
//    {
//        if (bufferDeProfundidade == NULL)
//        {
//            IFrameDescription* frameDescription = NULL;
//            hr = depthFrame->get_FrameDescription(&frameDescription);
//            frameDescription->get_Width(&depthFrameWidth);
//            frameDescription->get_Height(&depthFrameHeight);
//            bufferDeProfundidade = new UINT16[depthFrameHeight * depthFrameWidth];
//            depthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
//            depthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
//            SafeRelease(frameDescription);
//        }

//        UINT bufferSize = depthFrameHeight * depthFrameWidth;
//        //hr = depthFrame->CopyFrameDataToArray(bufferSize, bufferDeProfundidade);
//        hr = depthFrame->AccessUnderlyingBuffer(&bufferSize, &bufferDeProfundidade);
//    }

//    SafeRelease(multiFrame);
//    SafeRelease(depthFrame);
//    SafeRelease(depthFrameReference);

//    return bufferDeProfundidade;
//}

UINT16* KinectSensor::getInfraredFrame()
{
    //Multi frame
    IMultiSourceFrame* multiFrame = NULL;
    HRESULT hr = msFrameReader->AcquireLatestFrame(&multiFrame);

    if (!SUCCEEDED(hr))
    {
        return NULL;
    }

    IInfraredFrameReference* infraredFrameReference = NULL;
    IInfraredFrame* pInfraredFrame = NULL;;
    multiFrame->get_InfraredFrameReference(&infraredFrameReference);
    hr = infraredFrameReference->AcquireFrame(&pInfraredFrame);
    UINT16 *pBuffer = NULL;

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        UINT nBufferSize = 0;


        hr = pInfraredFrame->get_RelativeTime(&nTime);

        if (SUCCEEDED(hr))
        {
            hr = pInfraredFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Width(&nWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Height(&nHeight);
        }

        if (SUCCEEDED(hr))
        {
            //hr = pInfraredFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
            hr = pInfraredFrame->CopyFrameDataToArray(nBufferSize,pBuffer);
            //CopyFrameDataToArray(nHeight * nWidth, irBuffer);
        }

        SafeRelease(pFrameDescription);
    }

    SafeRelease(multiFrame);
    SafeRelease(pInfraredFrame);
    SafeRelease(infraredFrameReference);

    return pBuffer;
}

void KinectSensor::MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
    pCoordinateMapper->MapDepthFrameToCameraSpace(depthFrameWidth * depthFrameHeight, bufferDeProfundidade, depthFrameWidth * depthFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
}

void KinectSensor::MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
    pCoordinateMapper->MapColorFrameToCameraSpace(depthFrameWidth * depthFrameHeight, bufferDeProfundidade, colorFrameWidth * colorFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
}

void KinectSensor::MapDepthFrameToColorSpace(Point2f *pColorSpacePoints)
{
    pCoordinateMapper->MapDepthFrameToColorSpace(depthFrameWidth * depthFrameHeight, bufferDeProfundidade, depthFrameWidth * depthFrameHeight, (ColorSpacePoint*)pColorSpacePoints);
}

void KinectSensor::MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints)
{
    pCoordinateMapper->MapColorFrameToDepthSpace(depthFrameWidth * depthFrameHeight, bufferDeProfundidade, colorFrameWidth * colorFrameHeight, (DepthSpacePoint*)pDepthSpacePoints);;
}
