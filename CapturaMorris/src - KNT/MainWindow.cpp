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

using namespace DirectX;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_nStartTime = 0;
    m_nLastCounter = 0;
    m_nFramesSinceUpdate = 0;
    m_fFreq = 0;

    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }

    InitializeDefaultSensor();

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(atualizarFrame()));
    timer->start(33);

    m_pInfraredRGBX         = new RGBQUAD[cInfraredWidth * cInfraredHeight];
    m_pDepthRGBX            = new RGBQUAD[cDepthWidth * cDepthHeight];
    m_pColorRGBX            = new RGBQUAD[cColorWidth * cColorHeight];
    m_p1920OutputRGBX       = new RGBQUAD[cColorWidth * cColorHeight];
    m_pCroppedOutputRGBX    = new RGBQUAD[cColorCroppedWidth * cColorHeight];
    m_p512OutputRGBX        = new RGBQUAD[cDepthWidth * cDepthHeight];
    m_pReadRGBX             = new RGBQUAD[cColorCroppedWidth * cColorHeight];
    m_pBackgroundRGBX       = new RGBQUAD[cDepthWidth * cDepthHeight];

    m_pDepthCoordinates     = new DepthSpacePoint[cColorWidth * cColorHeight];
    m_pColorCoordinates     = new ColorSpacePoint[cDepthWidth * cDepthHeight];

    connect(ui->btnAbrir, SIGNAL(clicked(bool)), this, SLOT(fileOpen()));
}

MainWindow::~MainWindow()
{
    delete ui;

    if (m_p512OutputRGBX)
    {
        delete [] m_p512OutputRGBX;
        m_p512OutputRGBX = NULL;
    }

    if (m_pDepthCoordinates)
    {
        delete[] m_pDepthCoordinates;
        m_pDepthCoordinates = NULL;
    }

    if (m_pInfraredRGBX)
    {
        delete [] m_pInfraredRGBX;
        m_pInfraredRGBX = NULL;
    }

    if (m_pColorRGBX)
    {
        delete [] m_pColorRGBX;
        m_pColorRGBX = NULL;
    }

    if (m_pDepthRGBX)
    {
        delete [] m_pDepthRGBX;
        m_pDepthRGBX = NULL;
    }

    SafeRelease(m_pMultiSourceFrameReader);
    SafeRelease(m_pCoordinateMapper);

    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

HRESULT MainWindow::InitializeDefaultSensor(){

    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->OpenMultiSourceFrameReader(
                        FrameSourceTypes::FrameSourceTypes_Depth |
                        FrameSourceTypes::FrameSourceTypes_Color |
                        FrameSourceTypes::FrameSourceTypes_Infrared,
                        &m_pMultiSourceFrameReader);
        }
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        return E_FAIL;
    }

    return hr;
}

void MainWindow::atualizarFrame()
{
    if (!m_pMultiSourceFrameReader)
    {
        return;
    }

    IMultiSourceFrame* pMultiSourceFrame = NULL;
    IDepthFrame* pDepthFrame = NULL;
    IColorFrame* pColorFrame = NULL;
    IInfraredFrame* pInfraredFrame = NULL;

    HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

    if (SUCCEEDED(hr))
    {
        IColorFrameReference* pColorFrameReference = NULL;

        hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
        if (SUCCEEDED(hr))
        {
            hr = pColorFrameReference->AcquireFrame(&pColorFrame);
        }

        SafeRelease(pColorFrameReference);
    }

    if (SUCCEEDED(hr))
    {
        IInfraredFrameReference* pInfraredFrameReference = NULL;

        hr = pMultiSourceFrame->get_InfraredFrameReference(&pInfraredFrameReference);
        if (SUCCEEDED(hr))
        {
            hr = pInfraredFrameReference->AcquireFrame(&pInfraredFrame);
        }

        SafeRelease(pInfraredFrameReference);
    }

    if (SUCCEEDED(hr))
    {
        IDepthFrameReference* pDepthFrameReference = NULL;

        hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
        }

        SafeRelease(pDepthFrameReference);
    }

    // color
    INT64 nColorTime = 0;
    IFrameDescription* pColorFrameDescription = NULL;
    int nColorWidth = 0;
    int nColorHeight = 0;
    ColorImageFormat imageFormat = ColorImageFormat_None;
    UINT nColorBufferSize = 0;
    RGBQUAD *pColorBuffer = NULL;

    if (SUCCEEDED(hr))
    {
        hr = pColorFrame->get_RelativeTime(&nColorTime);
    }

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
        hr = pColorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel);
    }

    if (SUCCEEDED(hr))
    {
        nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
        if (imageFormat == ColorImageFormat_Bgra)
        {
            hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
        }
        else if (m_pColorRGBX)
        {
            pColorBuffer = m_pColorRGBX;
            hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
        }
        else
        {
            hr = E_FAIL;
        }
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

        // process frames
        ProcessFrame(nColorTime, pDepthBuffer, nDepthWidth, nDepthHeight, pColorBuffer, nColorWidth, nColorHeight);
        //ProcessInfrared(nInfraredTime, pInfraredBuffer, nInfraredWidth, nInfraredHeight);
        //ProcessDepth(nDepthTime, pDepthBuffer, nDepthWidth, nDepthHeight, nDepthMinReliableDistance, nDepthMaxDistance);

        SafeRelease(pDepthFrameDescription);
        SafeRelease(pColorFrameDescription);
        SafeRelease(pInfraredFrameDescription);
    }

    SafeRelease(pDepthFrame);
    SafeRelease(pColorFrame);
    SafeRelease(pInfraredFrame);
    SafeRelease(pMultiSourceFrame);
}

void MainWindow::ProcessFrame(INT64 nTime, const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
                              const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight)
{

    FPS(ui->lblCor, nTime);

    if (m_pCoordinateMapper && m_pColorCoordinates && m_p512OutputRGBX &&
            pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) &&
            pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight))
    {
        HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(nDepthWidth * nDepthHeight,
                                                                    (UINT16*)pDepthBuffer, nDepthWidth * nDepthHeight, m_pColorCoordinates);

        if (SUCCEEDED(hr))
        {
            // Cropping and downsampling
            int downSampleFactor = 1;
            const unsigned int h = nColorHeight;
            int w = cColorCroppedWidth;
            int end = w + marginColorLeft;
            int depthIndex = 0;
            for (int y = 0; y < h; y += downSampleFactor)
            {
                for (int x = marginColorLeft; x < end; x += downSampleFactor)
                {
                    int colorIndex = (y * nColorWidth) + x;
                    const RGBQUAD* pSrc = &m_pColorRGBX[colorIndex];
                    m_pCroppedOutputRGBX[depthIndex] = *pSrc;
                    depthIndex++;
                }
            }

            // Mirroring
            Concurrency::parallel_for(0u, h, [&](unsigned int y)
            {
                unsigned int index = y * w;
                unsigned int mirrorIndex = index + w - 1;

                for (unsigned int x = 0; x < (w / 2); ++x, ++index, --mirrorIndex)
                {
                    RGBQUAD pixel = m_pCroppedOutputRGBX[index];
                    m_pCroppedOutputRGBX[index] = m_pCroppedOutputRGBX[mirrorIndex];
                    m_pCroppedOutputRGBX[mirrorIndex] = pixel;
                }
            });


            ui->lblCor->setPixmap(QPixmap::fromImage(QImage((uchar*)m_pCroppedOutputRGBX,
              cColorCroppedWidth, h, QImage::Format_RGB32)).scaledToWidth(projectionScale));


            bool gravarColor = ui->chkCor->isChecked();
            if (gravarColor){
                int rgbQuadSize = sizeof(RGBQUAD);
                QString nomeDoArquivo = ui->txtNome->toPlainText();
                UINT size = (cColorCroppedWidth * cColorHeight * rgbQuadSize);
                kinectWriter.writeColorFrame(nomeDoArquivo.toUtf8().constData(), m_pCroppedOutputRGBX, size);
            } else if (kinectWriter.openedForWriting()){
                kinectWriter.closeFileIfOpened();
            }
        }
    }
}

void MainWindow::ProcessInfrared(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight)
{

    FPS(ui->lblFPSIR, nTime);

    if (m_pInfraredRGBX && pBuffer && (nWidth == cInfraredWidth) && (nHeight == cInfraredHeight))
    {
        RGBQUAD* pDest = m_pInfraredRGBX;
        const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

        while (pBuffer < pBufferEnd)
        {
            // normalize the incoming infrared data (ushort) to a float ranging from
            // [InfraredOutputValueMinimum, InfraredOutputValueMaximum] by
            // 1. dividing the incoming value by the source maximum value
            float intensityRatio = static_cast<float>(*pBuffer) / InfraredSourceValueMaximum;

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
            ++pBuffer;
        }

        unsigned int width = nWidth;
        unsigned int height = nHeight;
        RGBQUAD* rawPixels = m_pInfraredRGBX;

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

        ui->lblIR->setPixmap(QPixmap::fromImage(QImage((uchar*)m_pInfraredRGBX, cInfraredWidth,
               cInfraredHeight, QImage::Format_RGB32)).scaledToWidth(projectionScale));


        bool gravarInfraVermelho = ui->chkInfravermelho->isChecked();
        if (gravarInfraVermelho){
            QString nomeDoArquivo = ui->txtNome->toPlainText();
            UINT size = cInfraredWidth * cInfraredHeight * sizeof(RGBQUAD);
            kinectWriter.writeColorFrame(nomeDoArquivo.toUtf8().constData(), m_pInfraredRGBX, size);
        } else if (kinectWriter.openedForWriting()){
            kinectWriter.closeFileIfOpened();
        }
    }
}

void MainWindow::fileOpen()
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
        currentFileName = nomeDoArquivo;

        int totalDeBytes = cColorCroppedWidth * cColorHeight * sizeof(RGBQUAD);

        QThread *thread;
        thread = new QThread;
        KinectReader *kinectReader;
        std::string nome = currentFileName.toUtf8().constData();
        kinectReader = new KinectReader(nome, totalDeBytes,  m_pReadRGBX);
        kinectReader->moveToThread(thread);
        connect(thread, SIGNAL(started()), kinectReader, SLOT(processar()));
        connect(kinectReader, SIGNAL(processado()), this, SLOT(updateView()));
        connect(kinectReader, SIGNAL(concluido()), thread, SLOT(quit()));
        thread->start();
    }
}

void MainWindow::updateView()
{
    ui->lblPlay->setPixmap(QPixmap::fromImage(QImage((uchar*) m_pReadRGBX,
      cColorCroppedWidth, cColorHeight, QImage::Format_RGB32)).scaledToWidth(projectionScale));
}

void MainWindow::playerPlay()
{
    Q_EMIT play();
}


void MainWindow::playerStop()
{
    Q_EMIT stop();
}

void MainWindow::ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight)
{
    if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight))
    {
        unsigned int width = nWidth;
        unsigned int height = nHeight;
        RGBQUAD* rawPixels = pBuffer;

        Concurrency::parallel_for(0u, height, [&](unsigned int y)
        {
            unsigned int index = y * width;
            unsigned int mirrorIndex = index + width - 1;

            for (unsigned int x = 0; x < (width / 2); ++x, ++index, --mirrorIndex)
            {
                RGBQUAD pixel = rawPixels[index];
                rawPixels[index] = rawPixels[mirrorIndex];
                rawPixels[mirrorIndex] = pixel;
            }
        });

        ui->lblCor->setPixmap(QPixmap::fromImage(QImage((uchar*)pBuffer, cColorWidth,
                                                        cColorHeight, QImage::Format_RGB32)).scaledToWidth(projectionScale));
    }
}

void MainWindow::ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
    FPS(ui->lblFPS3D, nTime);

    if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
    {

        RGBQUAD* pRGBX = m_pDepthRGBX;
        const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

        while (pBuffer < pBufferEnd)
        {
            USHORT depth = *pBuffer;

            // To convert to a byte, we're discarding the most-significant
            // rather than least-significant bits.
            // We're preserving detail, although the intensity will "wrap."
            // Values outside the reliable depth range are mapped to 0 (black).

            // Note: Using conditionals in this loop could degrade performance.
            // Consider using a lookup table instead when writing production code.
            BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

            pRGBX->rgbRed   = intensity;
            pRGBX->rgbGreen = intensity;
            pRGBX->rgbBlue  = intensity;

            ++pRGBX;
            ++pBuffer;
        }

        unsigned int width = nWidth;
        unsigned int height = nHeight;
        RGBQUAD* rawPixels = m_pDepthRGBX;

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


        ui->lbl3D->setPixmap(QPixmap::fromImage(QImage((uchar*)m_pDepthRGBX, cDepthWidth,
                                                       cDepthHeight, QImage::Format_RGB32)).scaledToWidth(projectionScale));

        bool gravar3d = ui->chk3d->isChecked();
        if (gravar3d){
            QString nomeDoArquivo = ui->txtNome->toPlainText();
            UINT size = cDepthWidth * cDepthHeight * sizeof(RGBQUAD);
            kinectWriter.writeColorFrame(nomeDoArquivo.toUtf8().constData(), m_pDepthRGBX, size);
        } else if (kinectWriter.openedForWriting()){
            kinectWriter.closeFileIfOpened();
        }
    }
}

void MainWindow::FPS(QLabel *label, INT64 nTime)
{

    if (!m_nStartTime)
    {
        m_nStartTime = nTime;
    }

    double fps = 0.0;

    LARGE_INTEGER qpcNow = {0};
    if (m_fFreq)
    {
        if (QueryPerformanceCounter(&qpcNow))
        {
            if (m_nLastCounter)
            {
                m_nFramesSinceUpdate++;
                fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);


            }
        }
    }

    std::string str = (boost::format("FPS: %1$.0f ") % fps).str();
    label->setText(QString(str.c_str()));

    INT64 now = GetTickCount64();
    if (m_nNextStatusTime <= now)
    {
        m_nNextStatusTime = now + 1000; //1s
        m_nLastCounter = qpcNow.QuadPart;
        m_nFramesSinceUpdate = 0;
    }
}
