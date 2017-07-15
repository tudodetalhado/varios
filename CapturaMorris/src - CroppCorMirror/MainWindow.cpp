#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QTimer>
#include <QFileDialog>

#pragma warning(push)
#pragma warning(disable:6255)
#pragma warning(disable:6263)
#pragma warning(disable:4995)
#include "ppl.h"
#pragma warning(pop)

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->lcdDisplay->display(QTime(0, 0).toString());
    ui->lblFPS->setStyleSheet("color: red");

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
    timer->start(34);

    _1920x1080_rgb_buffer = new RGBQUAD[_1920 * _1080];
    _1080x1080_rgb_buffer = new RGBQUAD[_1080 * _1080];

    _512x424_int_buffer = new UINT16[_512 * _424];
    _512x424_rgb_buffer = new RGBQUAD[_512 * _424];
    _424x424_rgb_buffer = new RGBQUAD[_424 * _424];

    _depthBuffer = new RGBQUAD[_512 * _424];
    _tempDepthBuffer  = new RGBQUAD[_512 * _424];

    _croppedBuffer = new RGBQUAD[424 * 424];
    bufferLeitura = new RGBQUAD[colorWidthCropped * _1080];

    connect(ui->btnVisualizar, SIGNAL(clicked(bool)), this, SLOT(abrirTrack()));
    connect(ui->btnGravar, SIGNAL(clicked(bool)), this, SLOT(gravarTrack()));
    connect(ui->chkPararAuto, SIGNAL(clicked(bool)), this, SLOT(setPararGravacaoAuto()));
    connect(&_timer, SIGNAL(timeout()), this, SLOT(atualizarLCD()));
    connect(ui->tabViews, SIGNAL(currentChanged(int)), this, SLOT(alterarStatus()));
}

void MainWindow::alterarStatus()
{
    if(ui->tabViews->currentIndex() == 0)
    {
       _deveProcessar = true;
    }
    else if(ui->tabViews->currentIndex() == 1)
    {
       _deveProcessar = false;
    }
}

void MainWindow::setPararGravacaoAuto()
{
    if (_pararGravacaoAuto == true)
    {
        ui->chkPararAuto->setChecked(false);
        _pararGravacaoAuto = false;
    }
    else
    {
        ui->chkPararAuto->setChecked(true);
        _pararGravacaoAuto = true;
    }

}

void MainWindow::atualizarLCD()
{
    ui->lcdDisplay->display(_cron.getTime().toString());
    QTime cronTime = _cron.getTime();
    QTime maxTime = ui->cboTimerLimite->time();
    if (_pararGravacaoAuto == true && cronTime.operator >=(maxTime)){
        definirTela();
    }
}

void MainWindow::gravarTrack()
{
    _deveProcessar = true;

    BOOLEAN isSensorConectado = false;
    HRESULT hr = sensorKinect->get_IsAvailable(&isSensorConectado);
    if (isSensorConectado == true)
    {
        if (_deveGravar == true)
        {
            definirTela();
        }
        else
        {
            ui->btnGravar->setText("Parar");
            ui->btnGravar->setStyleSheet("background-color: red");
            ui->lcdDisplay->display(QTime(0, 0).toString());
            _deveGravar = true;
            _cron.reiniciar();
            _timer.start(1000);
        }
    }
}

void MainWindow::definirTela()
{
    ui->btnGravar->setText("Gravar");
    ui->btnGravar->setStyleSheet("background-color: green");
    _deveGravar = false;
    _timer.stop();
}

void MainWindow::atualizarTela()
{
    ui->lblViewTracks->setPixmap(QPixmap::fromImage(QImage((uchar*) bufferLeitura,
      colorWidthCropped, _1080, QImage::Format_RGB32)).scaledToWidth(imgScale));
}

void MainWindow::abrirTrack()
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

        int totalDeBytes = colorWidthCropped * _1080 * sizeof(RGBQUAD);

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
                fps = frequencia * totalDeframesAtualizados
                      / double(qpcNow.QuadPart - ultimaContagem);
            }
        }
    }

    if (fps < 20)
    {
        label->setStyleSheet("color: red");
    }
    else
    {
        label->setStyleSheet("color: green");
    }

    std::string str = (boost::format("FPS %1$.0f ") % fps).str();
    label->setText(QString(str.c_str()));

    INT64 now = GetTickCount64();
    if (proximaAtualizacaoTime <= now)
    {
        proximaAtualizacaoTime = now + 1000; //1s
        ultimaContagem = qpcNow.QuadPart;
        totalDeframesAtualizados = 0;
    }
}

HRESULT MainWindow::inicializarSensorKinect()
{
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
    if (!msFrameReader || !_deveProcessar)
    {
        return;
    }

    IMultiSourceFrame* multiSrcFrm = NULL;
    HRESULT hr = msFrameReader->AcquireLatestFrame(&multiSrcFrm);

    if (SUCCEEDED(hr))
    {
        // Processar cor
        bool deveProcessarCor = ui->rdb2Dcinza->isChecked();
        if (deveProcessarCor)
        {
            IColorFrameReference* colorFrmRef = NULL;
            hr = multiSrcFrm->get_ColorFrameReference(&colorFrmRef);

            if (SUCCEEDED(hr))
            {
                IColorFrame* colorFrm = NULL;
                hr = colorFrmRef->AcquireFrame(&colorFrm);

                if (SUCCEEDED(hr))
                {
                    IFrameDescription* colorFrmDesc = NULL;

                    //ColorImageFormat imageFormat = ColorImageFormat_Bgra;
                    ColorImageFormat imageFormat = ColorImageFormat_None;

                    hr = colorFrm->get_RelativeTime(&_colorTime);
                    hr = colorFrm->get_RawColorImageFormat(&imageFormat);
                    hr = colorFrm->get_FrameDescription(&colorFrmDesc);

                    if (SUCCEEDED(hr))
                    {
                        //verificar se vou precisar...
                        hr = colorFrmDesc->get_BytesPerPixel(&m_colorBytesPerPixel);
                        UINT colorBufferSize = _1920 * _1080 * rgbQuadSize;

                        if (imageFormat == ColorImageFormat_Bgra)
                        {
                            hr = colorFrm->AccessRawUnderlyingBuffer(&colorBufferSize, reinterpret_cast<BYTE**>(&_1920x1080_rgb_buffer));
                        }
                        else
                        {
                            hr = colorFrm->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(_1920x1080_rgb_buffer), ColorImageFormat_Bgra);
                        }
                        processarCor();
                    }
                    SafeRelease(colorFrmDesc);
                }
                SafeRelease(colorFrm);
            }
            SafeRelease(colorFrmRef);
        }

        // Processar frame infravermelho
        bool deveProcessarInfraVermelho = ui->rdb2Dinfra->isChecked();
        if (deveProcessarInfraVermelho){

            IInfraredFrameReference* infraFrmRef = NULL;
            hr = multiSrcFrm->get_InfraredFrameReference(&infraFrmRef);

            if (SUCCEEDED(hr))
            {
                IInfraredFrame* infraFrm = NULL;
                hr = infraFrmRef->AcquireFrame(&infraFrm);

                if (SUCCEEDED(hr))
                {
                    UINT infraBufferSize = 0;
                    hr = infraFrm->get_RelativeTime(&_infraTime);
                    hr = infraFrm->AccessUnderlyingBuffer(&infraBufferSize, &_512x424_int_buffer);
                    processarInfrared();
                }
                SafeRelease(infraFrm);
            }
            SafeRelease(infraFrmRef);
        }

        // Processar frame tridimensional
        bool deveProcessar3d = ui->rdb3Dcinza->isChecked();
        if (deveProcessar3d)
        {
            IDepthFrameReference* depthFrmRef = NULL;
            hr = multiSrcFrm->get_DepthFrameReference(&depthFrmRef);

            if (SUCCEEDED(hr))
            {
                int depthWidth = 0;
                int depthHeight = 0;
                INT64 depthTime = 0;
                UINT depthBufferSize = 0;
                UINT16 *depthBuffer = NULL;
                IFrameDescription* depthFrmDesc = NULL;
                USHORT minDepthReliableDistance = 0;
                USHORT maxDepthReliableDistance = 0;

                IDepthFrame* depthFrm = NULL;
                hr = depthFrmRef->AcquireFrame(&depthFrm);

                if (SUCCEEDED(hr))
                {
                    hr = depthFrm->get_RelativeTime(&depthTime);
                    hr = depthFrm->get_DepthMinReliableDistance(&minDepthReliableDistance);

                    // In order to see the full range of depth (including the less reliable far field depth)
                    // we are setting nDepthMaxDistance to the extreme potential depth threshold
                    maxDepthReliableDistance = USHRT_MAX;

                    // Note:  If you wish to filter by reliable depth distance, uncomment the following line.
                    //// hr = depthFrame->get_DepthMaxReliableDistance(&maxDepthReliableDistance);

                    hr = depthFrm->AccessUnderlyingBuffer(&depthBufferSize, &depthBuffer);
                    hr = depthFrm->get_FrameDescription(&depthFrmDesc);

                    if (SUCCEEDED(hr))
                    {
                        hr = depthFrmDesc->get_Width(&depthWidth);
                        hr = depthFrmDesc->get_Height(&depthHeight);
                        processarDepth(depthTime, depthBuffer, depthWidth, depthHeight,
                                       minDepthReliableDistance, maxDepthReliableDistance);
                    }
                    SafeRelease(depthFrmDesc);
                }
                SafeRelease(depthFrm);
            }
            SafeRelease(depthFrmRef);
        }

        if (_deveGravar)
        {
            //gravar();

            QString nomeDoArquivo = ui->txtId->toPlainText();

            bool gravarCor = ui->rdb2Dcinza->isChecked();
            if (gravarCor)
            {
                UINT size = (colorWidthCropped * _1080 * rgbQuadSize);
                kinectWriter.gravarFrame(nomeDoArquivo.toUtf8().constData(), _croppedBuffer, size);
            }

            bool gravarInfraVermelho = ui->rdb2Dinfra->isChecked();
            if (gravarInfraVermelho)
            {
                UINT size = _512 * _424 * rgbQuadSize;
                kinectWriter.gravarFrame(nomeDoArquivo.toUtf8().constData(), _512x424_rgb_buffer, size);
            }

            bool gravar3d = ui->rdb3Dcinza->isChecked();
            if (gravar3d){
                UINT size = _512 * _424 * rgbQuadSize;
                kinectWriter.gravarFrame(nomeDoArquivo.toUtf8().constData(), _depthBuffer, size);
            }
        }
        else if (kinectWriter.isAbertoParaGravar()){
            kinectWriter.fecharArquivoSeAberto();
        }
    }
    else
    {
       ui->lblFPS->setStyleSheet("color: red");
       ui->lblFPS->setText("Offline");
    }
    SafeRelease(multiSrcFrm);
}

void croppingFrame(int inWidth, RGBQUAD* inBuffer, int outWidth,
       int outHeight, RGBQUAD* outBuffer, int xOffSet, int yOffSet)
{
    Concurrency::parallel_for(yOffSet, (outHeight + yOffSet), [&](int y)
    {
        int outIndex = outWidth * y;

        for (int x = xOffSet; x < (outWidth + xOffSet); ++x, ++outIndex)
        {
            int inIndex = (y * inWidth) + x;
            outBuffer[outIndex] = inBuffer[inIndex];
        }
    });
}

void downsamplingFrame(int inWidth, RGBQUAD* inBuffer,
       int outWidth, int outHeight, RGBQUAD* outBuffer)
{
    float fator = (float)inWidth / (float)outWidth;
    Concurrency::parallel_for(0, outHeight, [&](int y)
    {
        unsigned int index = outWidth * y;
        for (int x = 0; x < outWidth; ++x, ++index)
        {
            int srcX = (int)(x * fator);
            int srcY = (int)(y * fator);
            int srcIndex = (srcY * inWidth) + srcX;
            outBuffer[index] = inBuffer[srcIndex];
        }
    });
}

void mirroringFrame(int width, int height, RGBQUAD* buffer)
{
    Concurrency::parallel_for(0, height, [&](int y)
    {
        int index = y * width;
        int mirrorIndex = index + width - 1;

        for (int x = 0; x < (width / 2); ++x, ++index, --mirrorIndex)
        {
            RGBQUAD pixel = buffer[index];
            buffer[index] = buffer[mirrorIndex];
            buffer[mirrorIndex] = pixel;
        }
    });
}

void MainWindow::processarCor()
{
    FPS(ui->lblFPS, _colorTime);

    int margin = (_1920 - _1080) / 2;
    croppingFrame(_1920, _1920x1080_rgb_buffer, _1080, _1080, _1080x1080_rgb_buffer, margin, 0);
    downsamplingFrame(_1080, _1080x1080_rgb_buffer, _424, _424, _424x424_rgb_buffer);
    mirroringFrame(_424, _424, _424x424_rgb_buffer);

    ui->lblViewGravacao->setPixmap(QPixmap::fromImage(QImage((uchar*)
        _424x424_rgb_buffer, _424, _424, QImage::Format_RGB32)));
}

void croppingFrameInfra(int inWidth, UINT16* inBuffer, int outWidth,
       int outHeight, UINT16* outBuffer, int xOffSet, int yOffSet)
{
    Concurrency::parallel_for(yOffSet, (outHeight + yOffSet), [&](int y)
    {
        int outIndex = outWidth * y;

        for (int x = xOffSet; x < (outWidth + xOffSet); ++x, ++outIndex)
        {
            int inIndex = (y * inWidth) + x;
            outBuffer[outIndex] = inBuffer[inIndex];
        }
    });
}

void MainWindow::processarInfrared()
{
    FPS(ui->lblFPS, _infraTime);

    RGBQUAD* tempBuffer = _512x424_rgb_buffer;
    const UINT16* totalBuffer = _512x424_int_buffer + (_424 * _512);

    while (_512x424_int_buffer < totalBuffer)
    {
        // normalize the incoming infrared data (ushort) to a float ranging from
        // [InfraredOutputValueMinimum, InfraredOutputValueMaximum] by
        // 1. dividing the incoming value by the source maximum value
        float intensityRatio = static_cast<float>(*_512x424_int_buffer) / InfraredSourceValueMaximum;

        // 2. dividing by the (average scene value * standard deviations)
        intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviations;

        // 3. limiting the value to InfraredOutputValueMaximum
        intensityRatio = min(InfraredOutputValueMaximum, intensityRatio);

        // 4. limiting the lower value InfraredOutputValueMinimym
        intensityRatio = max(InfraredOutputValueMinimum, intensityRatio);

        // 5. converting the normalized value to a byte and using the result
        // as the RGB components required by the image
        byte intensity = static_cast<byte>(intensityRatio * 255.0f);
        tempBuffer->rgbRed   = intensity;
        tempBuffer->rgbGreen = intensity;
        tempBuffer->rgbBlue  = intensity;

        ++tempBuffer;
        ++_512x424_int_buffer;
    }

    mirroringFrame(_512, _424, _512x424_rgb_buffer);

    ui->lblViewGravacao->setPixmap(QPixmap::fromImage(QImage((uchar*)_512x424_rgb_buffer,
          _512, _424, QImage::Format_RGB32)).scaledToWidth(400));
}

void MainWindow::processarDepth(INT64 depthTime, const UINT16* depthBuffer, int depthWidth, int depthHeight, USHORT minDepth, USHORT maxDepth)
{
    FPS(ui->lblFPS, depthTime);

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

    ui->lblViewGravacao->setPixmap(QPixmap::fromImage(QImage((uchar*)_depthBuffer, depthWidth,
        depthHeight, QImage::Format_RGB32)).scaledToWidth(imgScale));
}

MainWindow::~MainWindow()
{
    delete ui;

    if (bufferLeitura)
    {
        delete [] bufferLeitura;
        bufferLeitura = NULL;
    }

    if (_1920x1080_rgb_buffer)
    {
        delete [] _1920x1080_rgb_buffer;
        _1920x1080_rgb_buffer = NULL;
    }

    if (_512x424_int_buffer)
    {
        delete [] _512x424_int_buffer;
        _512x424_int_buffer = NULL;
    }

    if (_depthBuffer)
    {
        delete [] _depthBuffer;
        _depthBuffer = NULL;
    }

    if (_croppedBuffer)
    {
        delete [] _croppedBuffer;
        _croppedBuffer = NULL;
    }

    SafeRelease(msFrameReader);
    SafeRelease(mapper);

    if (sensorKinect)
    {
        sensorKinect->Close();
    }

    SafeRelease(sensorKinect);
}
