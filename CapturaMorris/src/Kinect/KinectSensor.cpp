
#include "KinectSensor.h"
#include "Common.h"

using namespace std;

KinectSensor::KinectSensor(int milliseconds) : QThread()
{
    //this->_circularBuffer = circularBuffer;
    this->milliseconds = milliseconds;
    kinectSensor = NULL;
    pCoordinateMapper = NULL;
    msFrameReader = NULL;
    bufferDeProfundidade = NULL;

    int scale = 1;
    cv::Size sz(1920/scale,1080/scale);
    char now[1024];
    time_t t = time(NULL);
    struct tm lnow;
    localtime_s(&lnow, &t);
    sprintf_s(now, "%04d-%02d-%02d_%02d-%02d-%02d%s", lnow.tm_year + 1900,
    lnow.tm_mon + 1, lnow.tm_mday, lnow.tm_hour, lnow.tm_min, lnow.tm_sec, ".mp4");
    //_videoWriter = cv::VideoWriter(now()+".avi",CV_FOURCC_MACRO('X','V','I','D'), 30.0, sz);
    //_videoWriter = cv::VideoWriter(now()+".mp4",CV_FOURCC_MACRO('M','P','4','V'), 30.0, sz);
    _videoWriter = cv::VideoWriter(now,CV_FOURCC_MACRO('F','M','P','4'), 30.0, sz);

    doStop = false;

    //cor
    _1920x1080_cor_buffer = new RGBQUAD[_frmColorWidth * _frmColorHeight * sizeof(RGBQUAD)];
    _1304x1080_cor_buffer = new RGBQUAD[1304 * 1080 * sizeof(RGBQUAD)];
    _640x530_cor_buffer   = new RGBQUAD[640  *  530 * sizeof(RGBQUAD)];

    inicializar();
}

KinectSensor::~KinectSensor()
{

    _videoWriter.release();
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


class KinectWriter
{

public:
    void gravar(cv::VideoWriter videoWriter, cv::Mat frame) const
    {
        videoWriter << frame;
        frame.release();
    }
};

//void gravarFunc(const cv::VideoWriter *videoWriter, const cv::Mat frame)
//{
//    const cv::VideoWriter *vd = videoWriter;
//    vd.write(frame);
//}

void KinectSensor::run()
{
    while(1)
    {
        _1920x1080_cor_buffer = new RGBQUAD[_frmColorWidth * _frmColorHeight * sizeof(RGBQUAD)];
        // Stop thread if doStop=TRUE //
        doStopMutex.lock();
        if(doStop)
        {
            doStop = false;
            doStopMutex.unlock();
            break;
        }
        doStopMutex.unlock();

        // Synchronize with other streams (if enabled for this stream)
        //sharedImageBuffer->sync(1);

        //processingMutex.lock();


        if (capturarFrame())
        {
            KinectTime *time = new KinectTime();
            time->valor = _colorTime;

            // Synchronize with other streams (if enabled for this stream)
            //int deviceNumber = 1;
            //sharedImageBuffer->sync(deviceNumber);

            // Add frame to buffer
            //bool dropFrameIfBufferFull = true;
            //cv::Mat frmCor = new cv::Mat(1080, 1920, CV_8UC4, _1920x1080_cor_buffer);

            //cv::Mat frmCor(1080, 1920, CV_8UC4, _1920x1080_cor_buffer);

            // Try and acquire semaphore to add item
            //freeSlots->tryAcquire();

            // Add item to queue
            //_bufferProtect.lock();
            //_circularBuffer->push(frmCor.clone());
            //_bufferProtect.unlock();

            // Release semaphore
            //usedSlots->release();

             //sharedImageBuffer->getByDeviceNumber(deviceNumber)->add(
             //           frmCor, dropFrameIfBufferFull);

            emit kinectColorCallback(_1920x1080_cor_buffer, time);


            if (_deveGravar)
            {
//                cv::Mat frmCor(1080, 1920, CV_8UC4, _1920x1080_cor_buffer);
//                KinectWriter kinectWriter;
//                std::thread t(&KinectWriter::gravar, &kinectWriter, _videoWriter, frmCor);
//                t.join();
//                frmCor.release();
            }


            //delete [] _1920x1080_cor_buffer;
            //_1920x1080_cor_buffer = NULL;
            //emit kinectColorCallback(_640x530_cor_buffer, time);
        }
        //processingMutex.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));

    }

//    while(_deveCapturar)
//    {
//        if (capturarFrame())
//        {
//            KinectTime *time = new KinectTime();
//            time->valor = _colorTime;
//            emit kinectColorCallback(_1920x1080_cor_buffer, time);
//            //emit kinectColorCallback(_640x530_cor_buffer, time);
//        }
//        //QThread.msleep(milliseconds);
//         std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
//    }
}

void KinectSensor::gravarVideo(cv::Mat frm)
{
   _videoWriter << frm;
   frm.release();
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
                UINT colorBufferSize = _frmColorWidth * _frmColorHeight * sizeof(RGBQUAD);

                ColorImageFormat imageFormat = ColorImageFormat_None;
                hr = colorFrm->get_RelativeTime(&_colorTime);
                hr = colorFrm->get_RawColorImageFormat(&imageFormat);

                //colorFrm->CopyConvertedFrameDataToArray(uBufferSize, frameColor.data, ColorImageFormat_Bgra);
                //colorFrm->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()),
                //        &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra);

                if (imageFormat == ColorImageFormat_Bgra)
                {
                    hr = colorFrm->AccessRawUnderlyingBuffer(&colorBufferSize, reinterpret_cast<BYTE**>(&_1920x1080_cor_buffer));
                }
                else
                {
                    hr = colorFrm->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(_1920x1080_cor_buffer), ColorImageFormat_Bgra);
                }
                capturou = true;
                //processarCor();
            }
            SafeRelease(colorFrm);
        }
        SafeRelease(colorFrmRef);
    }
    SafeRelease(multiSrcFrm);

    //frameCropping();
    //frameDownsampling();
    //frameMirroring();

    return capturou;
}

void KinectSensor::frameCropping()
{
    //cropping
    int margin = (_frmColorWidth - 1304) / 2;
    Concurrency::parallel_for(0, _frmColorHeight, [&](int y)
    {
        int outIndex = 1304 * y;
        for (int x = margin; x < (1304 + margin); ++x, ++outIndex)
        {
            int inIndex = (y * _frmColorWidth) + x;
            _1304x1080_cor_buffer[outIndex] = _1920x1080_cor_buffer[inIndex];
        }
    });
}

void KinectSensor::frameDownsampling()
{
    //downsampling
    float fator = (float) _frmColorHeight / (float) 530;
    Concurrency::parallel_for(0, 530, [&](int y)
    {
        unsigned int index = 640 * y;
        for (int x = 0; x < 640; ++x, ++index)
        {
            int srcX = (int)(x * fator);
            int srcY = (int)(y * fator);
            int srcIndex = (srcY * 1304) + srcX;
            _640x530_cor_buffer[index] = _1304x1080_cor_buffer[srcIndex];
        }
    });
}

void KinectSensor::frameMirroring()
{
    //mirroring
    Concurrency::parallel_for(0, 530, [&](int y)
    {
        int index = y * 640;
        int mirrorIndex = index + 640 - 1;

        for (int x = 0; x < (640 / 2); ++x, ++index, --mirrorIndex)
        {
            RGBQUAD pixel = _640x530_cor_buffer[index];
            _640x530_cor_buffer[index] = _640x530_cor_buffer[mirrorIndex];
            _640x530_cor_buffer[mirrorIndex] = pixel;
        }
    });
}

void KinectSensor::stop()
{
    QMutexLocker locker(&doStopMutex);
    _videoWriter.release();
    _deveGravar = false;
    doStop = true;
}

//RGBQUAD *KinectSensor::getColorFrame(IMultiSourceFrame* multiFrame)
//{
//    //Multi frame
//    //IMultiSourceFrame* multiFrame = NULL;
//    HRESULT hr = msFrameReader->AcquireLatestFrame(&multiFrame);

//    if (!SUCCEEDED(hr))
//    {
//        return NULL;
//    }

//    IColorFrameReference* colorFrameReference = NULL;
//    IColorFrame* colorFrame = NULL;
//    multiFrame->get_ColorFrameReference(&colorFrameReference);
//    hr = colorFrameReference->AcquireFrame(&colorFrame);

//    if (SUCCEEDED(hr))
//    {
//        if (_1920x1080_cor_buffer == NULL)
//        {
//            IFrameDescription* pFrameDescription = NULL;
//            hr = colorFrame->get_FrameDescription(&pFrameDescription);
//            hr = pFrameDescription->get_Width(&colorFrameWidth);
//            hr = pFrameDescription->get_Height(&colorFrameHeight);
//            _1920x1080_cor_buffer = new RGBQUAD[colorFrameWidth * colorFrameHeight];
//            SafeRelease(pFrameDescription);
//        }

//        UINT nBufferSize = colorFrameWidth * colorFrameHeight * sizeof(RGBQUAD);
//        hr = colorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(_1920x1080_cor_buffer), ColorImageFormat_Bgra);
//    }

//    SafeRelease(multiFrame);
//    SafeRelease(colorFrame);
//    SafeRelease(colorFrameReference);

//    frameCropping();
//    frameDownsampling();
//    frameMirroring();

//    return _1920x1080_cor_buffer;
//}

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





//void KinectSensor::MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints)
//{
//    pCoordinateMapper->MapDepthFrameToCameraSpace(depthFrameWidth * depthFrameHeight, bufferDeProfundidade, depthFrameWidth * depthFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
//}

//void KinectSensor::MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints)
//{
//    pCoordinateMapper->MapColorFrameToCameraSpace(depthFrameWidth * depthFrameHeight, bufferDeProfundidade, _frmColorWidth * _frmColorHeight, (CameraSpacePoint*)pCameraSpacePoints);
//}

//void KinectSensor::MapDepthFrameToColorSpace(Point2f *pColorSpacePoints)
//{
//    pCoordinateMapper->MapDepthFrameToColorSpace(depthFrameWidth * depthFrameHeight, bufferDeProfundidade, depthFrameWidth * depthFrameHeight, (ColorSpacePoint*)pColorSpacePoints);
//}

//void KinectSensor::MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints)
//{
//    pCoordinateMapper->MapColorFrameToDepthSpace(depthFrameWidth * depthFrameHeight, bufferDeProfundidade, _frmColorWidth * _frmColorHeight, (DepthSpacePoint*)pDepthSpacePoints);;
//}


//HRESULT MainWindow::inicializarSensorKinect()
//{
//    HRESULT hr;

////    hr = GetDefaultKinectSensor(&sensorKinect);
////    if (FAILED(hr))
////    {
////        return hr;
////    }

////    if (sensorKinect)
////    {
////        hr = sensorKinect->get_CoordinateMapper(&mapper);
////        hr = sensorKinect->Open();
////        hr = sensorKinect->OpenMultiSourceFrameReader(
////                FrameSourceTypes::FrameSourceTypes_Depth |
////                FrameSourceTypes::FrameSourceTypes_Color |
////                FrameSourceTypes::FrameSourceTypes_Infrared,
////                &msFrameReader);
////    }

////    if (!sensorKinect || FAILED(hr))
////    {
////        return E_FAIL;
////    }

//    return hr;
//}

//void MainWindow::lerDadosSensorKinect()
//{
//    if (!msFrameReader || !_deveProcessar)
//    {
//        return;
//    }

//    IMultiSourceFrame* multiSrcFrm = NULL;
//    HRESULT hr = msFrameReader->AcquireLatestFrame(&multiSrcFrm);

//    if (SUCCEEDED(hr))
//    {
//        // Processar cor
//        bool deveProcessarCor = true; //ui->rdb2Dcinza->isChecked();
//        if (deveProcessarCor)
//        {
//            IColorFrameReference* colorFrmRef = NULL;
//            hr = multiSrcFrm->get_ColorFrameReference(&colorFrmRef);

//            if (SUCCEEDED(hr))
//            {
//                IColorFrame* colorFrm = NULL;
//                hr = colorFrmRef->AcquireFrame(&colorFrm);

//                if (SUCCEEDED(hr))
//                {
//                    UINT colorBufferSize = 1920 * 1080 * sizeof(RGBQUAD);
//                    UINT uBufferSize = 1080 * 1920 * 4 * sizeof(BYTE);

//                    ColorImageFormat imageFormat = ColorImageFormat_None;
//                    hr = colorFrm->get_RelativeTime(&_colorTime);
//                    hr = colorFrm->get_RawColorImageFormat(&imageFormat);

//                    //colorFrm->CopyConvertedFrameDataToArray(uBufferSize, frameColor.data, ColorImageFormat_Bgra);
//                    //colorFrm->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()),
//                    //        &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra);

//                    if (imageFormat == ColorImageFormat_Bgra)
//                    {
//                        hr = colorFrm->AccessRawUnderlyingBuffer(&colorBufferSize, reinterpret_cast<BYTE**>(&_1920x1080_cor_buffer));
//                    }
//                    else
//                    {
//                        hr = colorFrm->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(_1920x1080_cor_buffer), ColorImageFormat_Bgra);
//                    }
//                    processarCor();
//                }
//                SafeRelease(colorFrm);
//            }
//            SafeRelease(colorFrmRef);
//        }

//        // Processar frame infravermelho
//        bool deveProcessarInfraVermelho = false; //ui->rdb2Dinfra->isChecked();
//        if (deveProcessarInfraVermelho){

//            IInfraredFrameReference* infraFrmRef = NULL;
//            hr = multiSrcFrm->get_InfraredFrameReference(&infraFrmRef);

//            if (SUCCEEDED(hr))
//            {
//                IInfraredFrame* infraFrm = NULL;
//                hr = infraFrmRef->AcquireFrame(&infraFrm);

//                if (SUCCEEDED(hr))
//                {
//                    UINT infraBufferSize = 0;
//                    hr = infraFrm->get_RelativeTime(&_infraTime);
//                    hr = infraFrm->AccessUnderlyingBuffer(&infraBufferSize, &_512x424_infraInt_buffer);
//                    processarInfrared();
//                }
//                SafeRelease(infraFrm);
//            }
//            SafeRelease(infraFrmRef);
//        }

//        // Processar frame tridimensional
//        bool deveProcessar3d = false; //ui->rdb3Dcinza->isChecked();
//        if (deveProcessar3d)
//        {
//            IDepthFrameReference* depthFrmRef = NULL;
//            hr = multiSrcFrm->get_DepthFrameReference(&depthFrmRef);

//            if (SUCCEEDED(hr))
//            {
//                IDepthFrame* depthFrm = NULL;
//                hr = depthFrmRef->AcquireFrame(&depthFrm);

//                if (SUCCEEDED(hr))
//                {
//                    UINT depthBufferSize = 0;
//                    hr = depthFrm->get_RelativeTime(&_depthTime);
//                    hr = depthFrm->AccessUnderlyingBuffer(&depthBufferSize, &_512x424_depthInt_buffer);
//                    processarDepth();
//                }
//                SafeRelease(depthFrm);
//            }
//            SafeRelease(depthFrmRef);
//        }

//        if (_deveGravarVideo)
//        {
//            gravarVideo();
//        }
//        else if (kinectWriter.isAbertoParaGravar()){
//            kinectWriter.fecharArquivoSeAberto();
//        }
//    }
//    else
//    {
//       ui->lblFPS->setStyleSheet("color: red");
//       ui->lblFPS->setText("Offline");
//    }
//    SafeRelease(multiSrcFrm);
//}


//void MainWindow::processarCor()
//{
//    FPS(ui->lblFPS, _colorTime);

//    ArenaCena *arenaCena = new ArenaCena(this);

//    if (ui->viewCaptura->exibirCor())
//    {
//        //cropping
//        int margin = (1920 - 1304) / 2;
//        Concurrency::parallel_for(0, 1080, [&](int y)
//        {
//            int outIndex = 1304 * y;
//            for (int x = margin; x < (1304 + margin); ++x, ++outIndex)
//            {
//                int inIndex = (y * 1920) + x;
//                _1304x1080_cor_buffer[outIndex] = _1920x1080_cor_buffer[inIndex];
//            }
//        });

//        //downsampling
//        float fator = (float) 1080 / (float) 530;
//        Concurrency::parallel_for(0, 530, [&](int y)
//        {
//            unsigned int index = 640 * y;
//            for (int x = 0; x < 640; ++x, ++index)
//            {
//                int srcX = (int)(x * fator);
//                int srcY = (int)(y * fator);
//                int srcIndex = (srcY * 1304) + srcX;
//                _640x530_cor_buffer[index] = _1304x1080_cor_buffer[srcIndex];
//            }
//        });

//        //mirroring
//        Concurrency::parallel_for(0, 530, [&](int y)
//        {
//            int index = y * 640;
//            int mirrorIndex = index + 640 - 1;

//            for (int x = 0; x < (640 / 2); ++x, ++index, --mirrorIndex)
//            {
//                RGBQUAD pixel = _640x530_cor_buffer[index];
//                _640x530_cor_buffer[index] = _640x530_cor_buffer[mirrorIndex];
//                _640x530_cor_buffer[mirrorIndex] = pixel;
//            }
//        });

//        QImage qImage((uchar*)_640x530_cor_buffer, 640, 530, QImage::Format_RGB32);
//        QPixmap qPixmap = QPixmap::fromImage(qImage, Qt::ColorOnly);

//        arenaCena->addPixmap(qPixmap);
//    }
//    else
//    {
//        // Converte para OpenCV;
//        cv::Mat frm(1080, 1920, CV_8UC4, _1920x1080_cor_buffer);

//        //cv::Mat frm;
//        //cor.convertTo(frm, CV_8UC1);

//        // Converte para cinza;
//        cv::cvtColor(frm, frm, cv::COLOR_BGRA2GRAY);

//        /// Corta no comprimento;
//        //int largura = (1080 * 640) / 530;
//        //int margem = (1920 - largura) / 2;
//        //frm = frm(cv::Rect(margem, 0, largura, 1080));

//        /// Redimensiona proporcional na altura;
//        //cv::resize(frm, frm, cv::Size(640, 530));


//        // Redimensiona proporcional na altura;
//        int largura = (1920 * 530) / 1080;
//        cv::resize(frm, frm, cv::Size(largura, 530));

//        // Corta no comprimento;
//        int margem = (largura - 640) / 2;
//        frm = frm(cv::Rect(margem, 0, 640, 530));

//        // Espelha;
//        cv::flip(frm, frm, 1);

//        // Apresenta a imagem;
//        QImage qImage(frm.data, frm.cols, frm.rows,
//           static_cast<int>(frm.step), QImage::Format_Grayscale8);
//        QPixmap qPixmap = QPixmap::fromImage(qImage);
//        arenaCena->addPixmap(qPixmap);
//    }

//    int index = ui->tabViewPrincipal->currentIndex();
//    if (index == 0)
//    {
//        ui->viewCaptura->setCena(arenaCena);
//    }

//    if (index == 1)
//    {
//        ui->viewArena->setCena(arenaCena);
//    }
//}

//void MainWindow::processarInfrared()
//{
//    FPS(ui->lblFPS, _infraTime);

//    RGBQUAD* tempBuffer = _512x424_infraCor_buffer;
//    const UINT16* totalBuffer = _512x424_infraInt_buffer + (512 * 424);

//    //mapping
//    while (_512x424_infraInt_buffer < totalBuffer)
//    {
//        float intensityRatio = static_cast<float>(*_512x424_infraInt_buffer) / InfraredSourceValueMaximum;
//        intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviations;
//        intensityRatio = min(InfraredOutputValueMaximum, intensityRatio);
//        intensityRatio = max(InfraredOutputValueMinimum, intensityRatio);
//        byte intensity = static_cast<byte>(intensityRatio * 255.0f);
//        tempBuffer->rgbRed   = intensity;
//        tempBuffer->rgbGreen = intensity;
//        tempBuffer->rgbBlue  = intensity;
//        ++tempBuffer;
//        ++_512x424_infraInt_buffer;
//    }

//    //mirroring
//    Concurrency::parallel_for(0, 424, [&](int y)
//    {
//        int index = y * 512;
//        int mirrorIndex = index + 512 - 1;

//        for (int x = 0; x < (512 / 2); ++x, ++index, --mirrorIndex)
//        {
//            RGBQUAD pixel = _512x424_infraCor_buffer[index];
//            _512x424_infraCor_buffer[index] = _512x424_infraCor_buffer[mirrorIndex];
//            _512x424_infraCor_buffer[mirrorIndex] = pixel;
//        }
//    });
//    ui->viewCaptura->scene()->addPixmap(QPixmap::fromImage(QImage((uchar*)
//        _512x424_infraCor_buffer, 512, 424, QImage::Format_RGB32)));
//}

//void MainWindow::processarDepth()
//{

//    FPS(ui->lblFPS, _depthTime);

//    RGBQUAD* tempBuffer = _512x424_depthCor_buffer;
//    const UINT16* totalBuffer = _512x424_depthInt_buffer + (512 * 424);

//    while (_512x424_depthInt_buffer < totalBuffer)
//    {
//        USHORT depth = *_512x424_depthInt_buffer;

//        // To convert to a byte, we're discarding the most-significant
//        // rather than least-significant bits.
//        // We're preserving detail, although the intensity will "wrap."
//        // Values outside the reliable depth range are mapped to 0 (black).

//        // Note: Using conditionals in this loop could degrade performance.
//        // Consider using a lookup table instead when writing production code.
//        BYTE intensity = static_cast<BYTE>((depth >= _minDepth)
//                   && (depth <= _maxDepth) ? (depth % 256) : 0);

//        tempBuffer->rgbRed   = intensity;
//        tempBuffer->rgbGreen = intensity;
//        tempBuffer->rgbBlue  = intensity;

//        ++tempBuffer;
//        ++_512x424_depthInt_buffer;
//    }

//    //mirroring
//    Concurrency::parallel_for(0, 424, [&](int y)
//    {
//        int index = y * 512;
//        int mirrorIndex = index + 512 - 1;

//        for (int x = 0; x < (512 / 2); ++x, ++index, --mirrorIndex)
//        {
//            RGBQUAD pixel = _512x424_depthCor_buffer[index];
//            _512x424_depthCor_buffer[index] = _512x424_depthCor_buffer[mirrorIndex];
//            _512x424_depthCor_buffer[mirrorIndex] = pixel;
//        }
//    });

//    ui->viewCaptura->scene()->addPixmap(QPixmap::fromImage(QImage((uchar*)
//        _512x424_depthCor_buffer, 512, 424, QImage::Format_RGB32)));
//}
