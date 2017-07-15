#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QTimer>

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
    m_pKinectSensor(NULL),
    m_pDepthFrameReader(NULL),
    m_pDepthRGBX(NULL)
{
    ui->setupUi(this);

    // create heap storage for depth pixel data in RGBX format
    m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];

    inicializarSensor();

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(atualizarDados()));
    timer->start(33); //1000/33 = 30 fps
}

MainWindow::~MainWindow()
{
    delete ui;
    if (m_pDepthRGBX)
    {
        delete [] m_pDepthRGBX;
        m_pDepthRGBX = NULL;
    }

    // done with depth frame reader
    SafeRelease(m_pDepthFrameReader);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

HRESULT MainWindow::inicializarSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);

        // Initialize the Kinect and get the depth reader
        IDepthFrameSource* pDepthFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
        }

        SafeRelease(pDepthFrameSource);
    }

//    if (!m_pKinectSensor || FAILED(hr))
//    {
//        SetStatusMessage(L"No ready Kinect found!", 10000, true);
//        return E_FAIL;
//    }

    return hr;
}

void MainWindow::atualizarDados()
{
    if (!m_pDepthFrameReader)
    {
        return;
    }

    IDepthFrame* pDepthFrame = NULL;

    HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        USHORT nDepthMinReliableDistance = 0;
        USHORT nDepthMaxDistance = 0;
        UINT nBufferSize = 0;
        UINT16 *pBuffer = NULL;

        hr = pDepthFrame->get_RelativeTime(&nTime);

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Width(&nWidth);
            pFrameDescription->get_Width(&nDepthFrameWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Height(&nHeight);
            pFrameDescription->get_Height(&nDepthFrameHeight);
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
            hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
        }

        if (SUCCEEDED(hr))
        {
            ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
        }
    }

    SafeRelease(pDepthFrame);
}

void MainWindow::ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth,
                              int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
    // Make sure we've received valid data
    if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
    {
        RGBQUAD* pRGBX = m_pDepthRGBX;

        // end pixel is start + width*height - 1
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

        // Draw the data with Direct2D
        //m_pDrawDepth->Draw(reinterpret_cast<BYTE*>(m_pDepthRGBX), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));
        ui->lblVideo->setPixmap(QPixmap::fromImage(QImage((uchar*)m_pDepthRGBX, cDepthWidth,
                                cDepthHeight, QImage::Format_RGB32)).scaledToWidth(512));
    }
}
