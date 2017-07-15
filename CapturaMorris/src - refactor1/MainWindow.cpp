#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QTimer>
#include <DirectXMath.h>
#include <QFileDialog>

#pragma warning(push)
#pragma warning(disable:6255)
#pragma warning(disable:6263)
#pragma warning(disable:4995)
#include "ppl.h"
#pragma warning(pop)

//using namespace DirectX;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    inicializarSensorKinect();

    timeInicial = 0;
    ultimaContagem = 0;
    totalDeframesAtualizados = 0;
    frequencia = 0;

    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        frequencia = double(qpf.QuadPart);
    }

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(atualizarFrame()));
    timer->start(33);

    _colorBuffer = new RGBQUAD[colorWidth * colorHeight];
    _infraBuffer = new RGBQUAD[infraWidth * infraHeight];
    _depthBuffer = new RGBQUAD[depthWidth * depthHeight];

    croppedBuffer = new RGBQUAD[colorWidthCropped * colorHeight];
    bufferLeitura = new RGBQUAD[colorWidthCropped * colorHeight];

    connect(ui->btnAbrir, SIGNAL(clicked(bool)), this, SLOT(abrirArquivo()));
}

MainWindow::~MainWindow()
{
    delete ui;

    if (bufferLeitura)
    {
        delete [] bufferLeitura;
        bufferLeitura = NULL;
    }

    if (_colorBuffer)
    {
        delete [] _colorBuffer;
        _colorBuffer = NULL;
    }

    if (_infraBuffer)
    {
        delete [] _infraBuffer;
        _infraBuffer = NULL;
    }

    if (_depthBuffer)
    {
        delete [] _depthBuffer;
        _depthBuffer = NULL;
    }

    if (croppedBuffer)
    {
        delete [] croppedBuffer;
        croppedBuffer = NULL;
    }

    SafeRelease(msFrameReader);
    SafeRelease(mapper);

    if (sensorKinect)
    {
        sensorKinect->Close();
    }

    SafeRelease(sensorKinect);
}

HRESULT MainWindow::inicializarSensorKinect(){

    HRESULT hr;

    hr = GetDefaultKinectSensor(&sensorKinect);
    if (FAILED(hr))
    {
        return hr;
    }

    if (sensorKinect)
    {
        hr = sensorKinect->get_CoordinateMapper(&mapper);
        hr = sensorKinect->Open();
        hr = sensorKinect->OpenMultiSourceFrameReader(
                FrameSourceTypes::FrameSourceTypes_Depth |
                FrameSourceTypes::FrameSourceTypes_Color |
                FrameSourceTypes::FrameSourceTypes_Infrared,
                &msFrameReader);
    }

    if (!sensorKinect || FAILED(hr))
    {
        return E_FAIL;
    }

    return hr;
}

void MainWindow::atualizarFrame()
{
   if (!msFrameReader)
   {
       return;
   }

   IMultiSourceFrame* multiSrcFrm = NULL;
   IDepthFrame* pDepthFrame = NULL;
   IColorFrame* pColorFrame = NULL;
   IInfraredFrame* pInfraredFrame = NULL;

   HRESULT hr = msFrameReader->AcquireLatestFrame(&multiSrcFrm);

   if (SUCCEEDED(hr))
   {
       IColorFrameReference* pColorFrameReference = NULL;

       hr = multiSrcFrm->get_ColorFrameReference(&pColorFrameReference);
       if (SUCCEEDED(hr))
       {
           hr = pColorFrameReference->AcquireFrame(&pColorFrame);
       }

       SafeRelease(pColorFrameReference);
   }

   if (SUCCEEDED(hr))
   {
       IInfraredFrameReference* pInfraredFrameReference = NULL;

       hr = multiSrcFrm->get_InfraredFrameReference(&pInfraredFrameReference);
       if (SUCCEEDED(hr))
       {
           hr = pInfraredFrameReference->AcquireFrame(&pInfraredFrame);
       }

       SafeRelease(pInfraredFrameReference);
   }

   if (SUCCEEDED(hr))
   {
       IDepthFrameReference* pDepthFrameReference = NULL;

       hr = multiSrcFrm->get_DepthFrameReference(&pDepthFrameReference);
       if (SUCCEEDED(hr))
       {
           hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
       }

       SafeRelease(pDepthFrameReference);
   }




   if (SUCCEEDED(hr))
   {
       // infrared
       INT64 nInfraredTime = 0;
       IFrameDescription* pInfraredFrameDescription = NULL;
       int nInfraredWidth = 0;
       int nInfraredHeight = 0;
       UINT nInfraredBufferSize = 0;
       UINT16 *pInfraredBuffer = NULL;

       hr = pInfraredFrame->get_RelativeTime(&nInfraredTime);

       if (SUCCEEDED(hr))
       {
           hr = pInfraredFrame->get_FrameDescription(&pInfraredFrameDescription);
       }

       if (SUCCEEDED(hr))
       {
           hr = pInfraredFrameDescription->get_Width(&nInfraredWidth);
       }

       if (SUCCEEDED(hr))
       {
           hr = pInfraredFrameDescription->get_Height(&nInfraredHeight);
       }

       if (SUCCEEDED(hr))
       {
           hr = pInfraredFrame->AccessUnderlyingBuffer(&nInfraredBufferSize, &pInfraredBuffer);
       }

       // depth
       INT64 nDepthTime = 0;
       IFrameDescription* pDepthFrameDescription = NULL;
       int nDepthWidth = 0;
       int nDepthHeight = 0;
       UINT nDepthBufferSize = 0;
       UINT16 *pDepthBuffer = NULL;
       USHORT nDepthMinReliableDistance = 0;
       USHORT nDepthMaxDistance = 0;

       hr = pDepthFrame->get_RelativeTime(&nDepthTime);

       if (SUCCEEDED(hr))
       {
           hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
       }

       if (SUCCEEDED(hr))
       {
           hr = pDepthFrameDescription->get_Width(&nDepthWidth);
       }

       if (SUCCEEDED(hr))
       {
           hr = pDepthFrameDescription->get_Height(&nDepthHeight);
       }

       if (SUCCEEDED(hr))
       {
           hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
       }

       if (SUCCEEDED(hr))
       {
           // In order to see the full range of depth (including the less reliable far field depth)
           // we are setting nDepthMaxDistance to the extreme potential depth threshold
           nDepthMaxDistance = USHRT_MAX;

           // Note:  If you wish to filter by reliable depth distance, uncomment the following line.
           //// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
       }

       if (SUCCEEDED(hr))
       {
           hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
       }

       // color
       INT64 nColorTime = 0;
       IFrameDescription* pColorFrameDescription = NULL;
       int nColorWidth = 0;
       int nColorHeight = 0;
       ColorImageFormat imageFormat = ColorImageFormat_None;
       UINT nColorBufferSize = 0;
       RGBQUAD *pColorBuffer = NULL;


        hr = pColorFrame->get_RelativeTime(&nColorTime);


       if (SUCCEEDED(hr))
       {
           hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
       }

       if (SUCCEEDED(hr))
       {
           hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
       }

       if (SUCCEEDED(hr))
       {
           hr = pColorFrameDescription->get_Width(&nColorWidth);
       }

       if (SUCCEEDED(hr))
       {
           hr = pColorFrameDescription->get_Height(&nColorHeight);
       }

       if (SUCCEEDED(hr))
       {
           hr = pColorFrameDescription->get_BytesPerPixel(&m_colorBytesPerPixel);
       }

       if (SUCCEEDED(hr))
       {
           nColorBufferSize = colorWidth * colorHeight * sizeof(RGBQUAD);
           if (imageFormat == ColorImageFormat_Bgra)
           {
               hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
           }
           else if (_colorBuffer)
           {
               pColorBuffer = _colorBuffer;
               hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
           }
           else
           {
               hr = E_FAIL;
           }
       }

       // process frames
       processarCor(nColorTime, pColorBuffer, nColorWidth, nColorHeight);
       //processarInfrared(nInfraredTime, pInfraredBuffer, nInfraredWidth, nInfraredHeight);
       //processarDepth(nDepthTime, pDepthBuffer, nDepthWidth, nDepthHeight, nDepthMinReliableDistance, nDepthMaxDistance);

       SafeRelease(pDepthFrameDescription);
       SafeRelease(pColorFrameDescription);
       SafeRelease(pInfraredFrameDescription);
   }

   SafeRelease(pDepthFrame);
   SafeRelease(pColorFrame);
   SafeRelease(pInfraredFrame);
   SafeRelease(multiSrcFrm);
//    if (!msFrameReader)
//    {
//        return;
//    }

//    IMultiSourceFrame* multiSrcFrm = NULL;
//    HRESULT hr = msFrameReader->AcquireLatestFrame(&multiSrcFrm);

//    if (SUCCEEDED(hr))
//    {
//        // Processar frame de cor
//        IColorFrameReference* colorFrmRef = NULL;
//        hr = multiSrcFrm->get_ColorFrameReference(&colorFrmRef);

//        if (SUCCEEDED(hr))
//        {
//            IColorFrame* colorFrm = NULL;
//            hr = colorFrmRef->AcquireFrame(&colorFrm);

//            if (SUCCEEDED(hr))
//            {
//                int colorWidth = 0;
//                int colorHeight = 0;
//                INT64 colorTime = 0;
//                IFrameDescription* colorFrmDesc = NULL;

//                ColorImageFormat imageFormat = ColorImageFormat_None;
//                //ColorImageFormat imageFormat = ColorImageFormat_Bgra;

//                UINT colorBufferSize = 0;
//                RGBQUAD *colorBuffer = NULL;

//                hr = colorFrm->get_RelativeTime(&colorTime);
//                hr = colorFrm->get_RawColorImageFormat(&imageFormat);
//                hr = colorFrm->get_FrameDescription(&colorFrmDesc);

//                if (SUCCEEDED(hr))
//                {
//                    hr = colorFrmDesc->get_Width(&colorWidth);
//                    hr = colorFrmDesc->get_Height(&colorHeight);
//                    hr = colorFrmDesc->get_BytesPerPixel(&m_colorBytesPerPixel);

//                    colorBufferSize = colorWidth * colorHeight * sizeof(RGBQUAD);

//                    if (imageFormat == ColorImageFormat_Bgra)
//                    {
//                        hr = colorFrm->AccessRawUnderlyingBuffer(&colorBufferSize, reinterpret_cast<BYTE**>(&colorBuffer));
//                    }
//                    else if (_colorBuffer)
//                    {
//                        colorBuffer = _colorBuffer;
//                        hr = colorFrm->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(colorBuffer), ColorImageFormat_Bgra);
//                    }
//                    else
//                    {
//                        hr = E_FAIL;
//                    }
//                    ///processarCor(colorTime, colorBuffer, colorWidth, colorHeight);
//                }
//                SafeRelease(colorFrmDesc);
//            }
//            SafeRelease(colorFrm);
//        }
//        SafeRelease(colorFrmRef);

//        // Processar frame infravermelho
//        IInfraredFrameReference* infraFrmRef = NULL;
//        hr = multiSrcFrm->get_InfraredFrameReference(&infraFrmRef);

//        if (SUCCEEDED(hr))
//        {
//            IInfraredFrame* infraFrm = NULL;
//            hr = infraFrmRef->AcquireFrame(&infraFrm);

//            if (SUCCEEDED(hr))
//            {
//                int infraWidth = 0;
//                int infraHeight = 0;
//                INT64 infraTime = 0;
//                UINT infraBufferSize = 0;
//                UINT16 *infraBuffer = NULL;
//                IFrameDescription* infraFrmDesc = NULL;

//                hr = infraFrm->get_RelativeTime(&infraTime);
//                hr = infraFrm->get_FrameDescription(&infraFrmDesc);

//                if (SUCCEEDED(hr))
//                {
//                    hr = infraFrmDesc->get_Width(&infraWidth);
//                    hr = infraFrmDesc->get_Height(&infraHeight);

//                }
//                hr = infraFrm->AccessUnderlyingBuffer(&infraBufferSize, &infraBuffer);
//                processarInfrared(infraTime, infraBuffer, infraWidth, infraHeight);

//                SafeRelease(infraFrmDesc);
//            }
//            SafeRelease(infraFrm);
//        }
//        SafeRelease(infraFrmRef);

//        // Processar frame tridimensional
//        IDepthFrameReference* depthFrmRef = NULL;
//        hr = multiSrcFrm->get_DepthFrameReference(&depthFrmRef);

//        if (SUCCEEDED(hr))
//        {
//            int depthWidth = 0;
//            int depthHeight = 0;
//            INT64 depthTime = 0;
//            UINT depthBufferSize = 0;
//            UINT16 *depthBuffer = NULL;
//            IFrameDescription* depthFrmDesc = NULL;
//            USHORT minDepthReliableDistance = 0;
//            USHORT maxDepthReliableDistance = 0;

//            IDepthFrame* depthFrm = NULL;
//            hr = depthFrmRef->AcquireFrame(&depthFrm);

//            if (SUCCEEDED(hr))
//            {
//                hr = depthFrm->get_RelativeTime(&depthTime);
//                hr = depthFrm->get_DepthMinReliableDistance(&minDepthReliableDistance);

//                // In order to see the full range of depth (including the less reliable far field depth)
//                // we are setting nDepthMaxDistance to the extreme potential depth threshold
//                maxDepthReliableDistance = USHRT_MAX;

//                // Note:  If you wish to filter by reliable depth distance, uncomment the following line.
//                //// hr = depthFrame->get_DepthMaxReliableDistance(&maxDepthReliableDistance);

//                hr = depthFrm->AccessUnderlyingBuffer(&depthBufferSize, &depthBuffer);
//                hr = depthFrm->get_FrameDescription(&depthFrmDesc);

//                if (SUCCEEDED(hr))
//                {
//                    hr = depthFrmDesc->get_Width(&depthWidth);
//                    hr = depthFrmDesc->get_Height(&depthHeight);
//                    //processarDepth(depthTime, depthBuffer, depthWidth, depthHeight, minDepthReliableDistance, maxDepthReliableDistance);
//                }
//                SafeRelease(depthFrmDesc);
//            }
//            SafeRelease(depthFrm);
//        }
//        SafeRelease(depthFrmRef);
//    }
//    SafeRelease(multiSrcFrm);
}

void MainWindow::processarCor(INT64 colorTime, const RGBQUAD* colorBuffer, int colorWidth, int colorHeight)
{

    FPS(ui->lblFPSCor, colorTime);

    // Cropping and downsampling
    int fatorDeReducao = 1;
    const unsigned int tempHeight = colorHeight;
    int tempWidth = colorWidthCropped;
    int widthCalculado = tempWidth + mgColorLeft;

    const RGBQUAD* tempBuffer = colorBuffer;

    int depthIndex = 0;
    for (unsigned int y = 0; y < tempHeight; y += fatorDeReducao)
    {
        for (int x = mgColorLeft; x < widthCalculado; x += fatorDeReducao)
        {
            int colorIndex = (y * colorWidth) + x;
            const RGBQUAD* pSrc = &tempBuffer[colorIndex];
            croppedBuffer[depthIndex] = *pSrc;
            depthIndex++;
        }
    }

    // Mirroring
    Concurrency::parallel_for(0u, tempHeight, [&](unsigned int y)
    {
        unsigned int index = y * tempWidth;
        unsigned int mirrorIndex = index + tempWidth - 1;

        for (unsigned int x = 0; x < (tempWidth / 2); ++x, ++index, --mirrorIndex)
        {
            RGBQUAD pixel = croppedBuffer[index];
            croppedBuffer[index] = croppedBuffer[mirrorIndex];
            croppedBuffer[mirrorIndex] = pixel;
        }
    });

    ui->lblCor->setPixmap(QPixmap::fromImage(QImage((uchar*) croppedBuffer,
      colorWidthCropped, tempHeight, QImage::Format_RGB32)).scaledToWidth(imgScale));

    bool gravarCor = ui->chkCor->isChecked();
    if (gravarCor){

        int rgbQuadSize = sizeof(RGBQUAD);
        QString nomeDoArquivo = ui->txtNome->toPlainText();
        UINT size = (colorWidthCropped * colorHeight * rgbQuadSize);
        kinectWriter.gravarFrame(nomeDoArquivo.toUtf8().constData(), croppedBuffer, size);

    } else if (kinectWriter.isAbertoParaGravar()){

        kinectWriter.fecharArquivoSeAberto();
    }
}

void MainWindow::processarInfrared(INT64 infraTime, const UINT16* infraBuffer, int infraWidth, int infraHeight)
{
    FPS(ui->lblFPSIR, infraTime);

    if (_infraBuffer && infraBuffer && (infraWidth == infraWidth) && (infraHeight == infraHeight))
    {
        RGBQUAD* pDest = _infraBuffer;
        const UINT16* pBufferEnd = infraBuffer + (infraWidth * infraHeight);

        while (infraBuffer < pBufferEnd)
        {
            // normalize the incoming infrared data (ushort) to a float ranging from
            // [InfraredOutputValueMinimum, InfraredOutputValueMaximum] by
            // 1. dividing the incoming value by the source maximum value
            float intensityRatio = static_cast<float>(*infraBuffer) / InfraredSourceValueMaximum;

            // 2. dividing by the (average scene value * standard deviations)
            intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviations;

            // 3. limiting the value to InfraredOutputValueMaximum
            intensityRatio = min(InfraredOutputValueMaximum, intensityRatio);

            // 4. limiting the lower value InfraredOutputValueMinimym
            intensityRatio = max(InfraredOutputValueMinimum, intensityRatio);

            // 5. converting the normalized value to a byte and using the result
            // as the RGB components required by the image
            byte intensity = static_cast<byte>(intensityRatio * 255.0f);
            pDest->rgbRed = intensity;
            pDest->rgbGreen = intensity;
            pDest->rgbBlue = intensity;

            ++pDest;
            ++infraBuffer;
        }

        unsigned int width = infraWidth;
        unsigned int height = infraHeight;
        RGBQUAD* rawPixels = _infraBuffer;

        Concurrency::parallel_for(0u, height, [&](unsigned int y)
        {
            unsigned int index = y * width;
            unsigned int mirrorIndex = index + width - 1;

            for (unsigned int x = 0; x < (width / 2); ++x, ++index, --mirrorIndex)
            {
                // In-place swap to mirror
                RGBQUAD pixel = rawPixels[index];
                rawPixels[index] = rawPixels[mirrorIndex];
                rawPixels[mirrorIndex] = pixel;
            }
        });

        ui->lblIR->setPixmap(QPixmap::fromImage(QImage((uchar*)_infraBuffer, infraWidth,
               infraHeight, QImage::Format_RGB32)).scaledToWidth(imgScale));


        bool gravarInfraVermelho = ui->chkInfravermelho->isChecked();
        if (gravarInfraVermelho){
            QString nomeDoArquivo = ui->txtNome->toPlainText();
            UINT size = infraWidth * infraHeight * sizeof(RGBQUAD);
            kinectWriter.gravarFrame(nomeDoArquivo.toUtf8().constData(),_infraBuffer, size);
        } else if (kinectWriter.isAbertoParaGravar()){
            kinectWriter.fecharArquivoSeAberto();
        }
    }

//    FPS(ui->lblFPSIR, infraTime);

//    RGBQUAD* tempDest = _infraBuffer;
//    const UINT16* totalBuffer = infraBuffer + (infraHeight * infraWidth);

//    while (infraBuffer < totalBuffer)
//    {
//        // normalize the incoming infrared data (ushort) to a float ranging from
//        // [InfraredOutputValueMinimum, InfraredOutputValueMaximum] by
//        // 1. dividing the incoming value by the source maximum value
//        float intensityRatio = static_cast<float>(*infraBuffer) / InfraredSourceValueMaximum;

//        // 2. dividing by the (average scene value * standard deviations)
//        intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviations;

//        // 3. limiting the value to InfraredOutputValueMaximum
//        intensityRatio = min(InfraredOutputValueMaximum, intensityRatio);

//        // 4. limiting the lower value InfraredOutputValueMinimym
//        intensityRatio = max(InfraredOutputValueMinimum, intensityRatio);

//        // 5. converting the normalized value to a byte and using the result
//        // as the RGB components required by the image
//        byte intensity = static_cast<byte>(intensityRatio * 255.0f);
//        tempDest->rgbRed   = intensity;
//        tempDest->rgbGreen = intensity;
//        tempDest->rgbBlue  = intensity;

//        ++tempDest;
//        ++infraBuffer;
//    }

//    unsigned int tempWidth = infraHeight;
//    unsigned int tempHeight = infraWidth;
//    RGBQUAD* tempRawPixels = _infraBuffer;

//    Concurrency::parallel_for(0u, tempHeight, [&](unsigned int y)
//    {
//        unsigned int index = y * tempWidth;
//        unsigned int mirrorIndex = index + tempWidth - 1;

//        for (unsigned int x = 0; x < (tempWidth / 2); ++x, ++index, --mirrorIndex)
//        {
//            RGBQUAD pixel = tempRawPixels[index];
//            tempRawPixels[index] = tempRawPixels[mirrorIndex];
//            tempRawPixels[mirrorIndex] = pixel;
//        }
//    });

//    ui->lblIR->setPixmap(QPixmap::fromImage(QImage((uchar*)_infraBuffer, infraWidth,
//           infraHeight, QImage::Format_RGB32)).scaledToWidth(imgScale));


//    bool gravarInfraVermelho = ui->chkInfravermelho->isChecked();
//    if (gravarInfraVermelho){

//        int rgbQuadSize = sizeof(RGBQUAD);
//        QString nomeDoArquivo = ui->txtNome->toPlainText();
//        UINT size = infraWidth * infraHeight * rgbQuadSize;
//        kinectWriter.gravarFrame(nomeDoArquivo.toUtf8().constData(), _infraBuffer, size);

//    } else if (kinectWriter.isAbertoParaGravar()){

//        kinectWriter.fecharArquivoSeAberto();
//    }

}

void MainWindow::processarDepth(INT64 depthTime, const UINT16* depthBuffer, int depthWidth, int depthHeight, USHORT minDepth, USHORT maxDepth)
{
    FPS(ui->lblFPS3D, depthTime);

    RGBQUAD* tempBuffer = _depthBuffer;
    const UINT16* totalBuffer = depthBuffer + (depthWidth * depthHeight);

    while (depthBuffer < totalBuffer)
    {
        USHORT depth = *depthBuffer;

        // To convert to a byte, we're discarding the most-significant
        // rather than least-significant bits.
        // We're preserving detail, although the intensity will "wrap."
        // Values outside the reliable depth range are mapped to 0 (black).

        // Note: Using conditionals in this loop could degrade performance.
        // Consider using a lookup table instead when writing production code.
        BYTE intensity = static_cast<BYTE>((depth >= minDepth) && (depth <= maxDepth) ? (depth % 256) : 0);

        tempBuffer->rgbRed   = intensity;
        tempBuffer->rgbGreen = intensity;
        tempBuffer->rgbBlue  = intensity;

        ++tempBuffer;
        ++depthBuffer;
    }

    unsigned int tempWidth = depthWidth;
    unsigned int tempHeight = depthHeight;
    RGBQUAD* tempRawPixels = _depthBuffer;

    Concurrency::parallel_for(0u, tempHeight, [&](unsigned int y)
    {
        unsigned int index = y * tempWidth;
        unsigned int mirrorIndex = index + tempWidth - 1;

        for (unsigned int x = 0; x < (tempWidth / 2); ++x, ++index, --mirrorIndex)
        {
            RGBQUAD pixel = tempRawPixels[index];
            tempRawPixels[index] = tempRawPixels[mirrorIndex];
            tempRawPixels[mirrorIndex] = pixel;
        }
    });

    ui->lbl3D->setPixmap(QPixmap::fromImage(QImage((uchar*)_depthBuffer, depthWidth,
        depthHeight, QImage::Format_RGB32)).scaledToWidth(imgScale));

    bool gravar3d = ui->chk3d->isChecked();
    if (gravar3d){

        int rgbQuadSize = sizeof(RGBQUAD);
        QString nomeDoArquivo = ui->txtNome->toPlainText();
        UINT size = depthWidth * depthHeight * rgbQuadSize;
        kinectWriter.gravarFrame(nomeDoArquivo.toUtf8().constData(), _depthBuffer, size);

    } else if (kinectWriter.isAbertoParaGravar()){

        kinectWriter.fecharArquivoSeAberto();
    }
}

void MainWindow::abrirArquivo()
{
    QString filtros = " Kinect Stream (*.knt);;"
                      " Todos (*.*)";

    QString nomeDoArquivo = QFileDialog::getOpenFileName(
                this,
                tr("Abrir arquivo"),
                QDir::currentPath() + "\\tracks",
                filtros);

    if (!nomeDoArquivo.isEmpty())
    {
        nomeArquivoAtual = nomeDoArquivo;

        int totalDeBytes = colorWidthCropped * colorHeight * sizeof(RGBQUAD);

        QThread *thread;
        thread = new QThread;
        KinectReader *kinectReader;
        std::string nome = nomeArquivoAtual.toUtf8().constData();
        kinectReader = new KinectReader(nome, totalDeBytes,  bufferLeitura);
        kinectReader->moveToThread(thread);
        connect(thread, SIGNAL(started()), kinectReader, SLOT(processar()));
        connect(kinectReader, SIGNAL(processado()), this, SLOT(atualizarTela()));
        connect(kinectReader, SIGNAL(concluido()), thread, SLOT(quit()));
        thread->start();
    }
}

void MainWindow::atualizarTela()
{
    ui->lblPlay->setPixmap(QPixmap::fromImage(QImage((uchar*) bufferLeitura,
      colorWidthCropped, colorHeight, QImage::Format_RGB32)).scaledToWidth(imgScale));
}

void MainWindow::FPS(QLabel *label, INT64 time)
{

    if (!timeInicial)
    {
        timeInicial = time;
    }

    double fps = 0.0;

    LARGE_INTEGER qpcNow = {0};
    if (frequencia)
    {
        if (QueryPerformanceCounter(&qpcNow))
        {
            if (ultimaContagem)
            {
                totalDeframesAtualizados++;
                fps = frequencia * totalDeframesAtualizados / double(qpcNow.QuadPart - ultimaContagem);
            }
        }
    }

    std::string str = (boost::format("FPS: %1$.0f ") % fps).str();
    label->setText(QString(str.c_str()));

    INT64 now = GetTickCount64();
    if (proximaAtualizacaoTime <= now)
    {
        proximaAtualizacaoTime = now + 1000; //1s
        ultimaContagem = qpcNow.QuadPart;
        totalDeframesAtualizados = 0;
    }
}
