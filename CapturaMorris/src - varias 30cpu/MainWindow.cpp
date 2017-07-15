#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QTimer>
#include <DirectXMath.h>

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

    InitializeDefaultSensor();

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(atualizarFrame()));
    timer->start(33);

    m_pInfraredRGBX = new RGBQUAD[cInfraredWidth * cInfraredHeight];
    m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];
    m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
}

MainWindow::~MainWindow()
{
   delete ui;

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

   // done with frame reader
   SafeRelease(m_pInfraredFrameReader);
   SafeRelease(m_pColorFrameReader);
   SafeRelease(m_pDepthFrameReader);

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
        IInfraredFrameSource* pInfraredFrameSource = NULL;
        IColorFrameSource* pColorFrameSource = NULL;
        IDepthFrameSource* pDepthFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_InfraredFrameSource(&pInfraredFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pInfraredFrameSource->OpenReader(&m_pInfraredFrameReader);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
        }


        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
        }


        SafeRelease(pInfraredFrameSource);
        SafeRelease(pColorFrameSource);
        SafeRelease(pDepthFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        //SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

void MainWindow::atualizarFrame()
{
    if (!m_pInfraredFrameReader)
    {
        return;
    }

    IInfraredFrame* pInfraredFrame = NULL;

    HRESULT hr = m_pInfraredFrameReader->AcquireLatestFrame(&pInfraredFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        UINT nBufferSize = 0;
        UINT16 *pBuffer = NULL;

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
            hr = pInfraredFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
        }

        if (SUCCEEDED(hr))
        {
            ProcessInfrared(nTime, pBuffer, nWidth, nHeight);
        }

        SafeRelease(pFrameDescription);
    }

    SafeRelease(pInfraredFrame);

    if (!m_pColorFrameReader)
    {
        return;
    }

    IColorFrame* pColorFrame = NULL;

    HRESULT hr2 = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

    if (SUCCEEDED(hr2))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        ColorImageFormat imageFormat = ColorImageFormat_None;
        UINT nBufferSize = 0;
        RGBQUAD *pBuffer = NULL;

        hr2 = pColorFrame->get_RelativeTime(&nTime);

        if (SUCCEEDED(hr2))
        {
            hr2 = pColorFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr2))
        {
            hr2 = pFrameDescription->get_Width(&nWidth);
        }

        if (SUCCEEDED(hr2))
        {
            hr2 = pFrameDescription->get_Height(&nHeight);
        }

        if (SUCCEEDED(hr2))
        {
            hr2 = pColorFrame->get_RawColorImageFormat(&imageFormat);
        }

        if (SUCCEEDED(hr2))
        {
            if (imageFormat == ColorImageFormat_Bgra)
            {
                hr2 = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
            }
            else if (m_pColorRGBX)
            {
                pBuffer = m_pColorRGBX;
                nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
                hr2 = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
            }
            else
            {
                hr2 = E_FAIL;
            }
        }

        if (SUCCEEDED(hr2))
        {
            ProcessColor(nTime, pBuffer, nWidth, nHeight);
        }

        SafeRelease(pFrameDescription);
    }

    SafeRelease(pColorFrame);

    if (!m_pDepthFrameReader)
    {
        return;
    }

    IDepthFrame* pDepthFrame = NULL;

    hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

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
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Height(&nHeight);
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

        SafeRelease(pFrameDescription);
    }

    SafeRelease(pDepthFrame);
}

void MainWindow::ProcessInfrared(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight)
{
    if (m_pInfraredRGBX && pBuffer && (nWidth == cInfraredWidth) && (nHeight == cInfraredHeight))
    {
        RGBQUAD* pDest = m_pInfraredRGBX;

        // end pixel is start + width*height - 1
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
                                cInfraredHeight, QImage::Format_RGB32)).scaledToWidth(250));
    }
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
                // In-place swap to mirror
                RGBQUAD pixel = rawPixels[index];
                rawPixels[index] = rawPixels[mirrorIndex];
                rawPixels[mirrorIndex] = pixel;
            }
        });

        ui->lblCor->setPixmap(QPixmap::fromImage(QImage((uchar*)pBuffer, cColorWidth,
                                cColorHeight, QImage::Format_RGB32)).scaledToWidth(400));
    }
}

void MainWindow::ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
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
                                cDepthHeight, QImage::Format_RGB32)).scaledToWidth(250));
    }
}
