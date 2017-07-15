#include "MainWindow.h"
#include "ui_MainWindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
        inicializarSensor();

    colorMat = cv::Mat::zeros(1080, 1920, CV_8UC3);

}

void MainWindow::inicializarSensor()
{
    cv::setUseOptimized( true );

    // Sensor
    HRESULT hResult = S_OK;
    hResult = GetDefaultKinectSensor( &pSensor );
    if( FAILED( hResult ) ){
        std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
    }

    hResult = pSensor->Open();
    if( FAILED( hResult ) ){
        std::cerr << "Error : IKinectSensor::Open()" << std::endl;
    }

    // Source
    IColorFrameSource* pColorSource;
    hResult = pSensor->get_ColorFrameSource( &pColorSource );
    if( FAILED( hResult ) ){
        std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
    }

    // Reader
    hResult = pColorSource->OpenReader(&pColorReader);
    if(FAILED(hResult)){
        std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
    }

    // Description
    IFrameDescription* pDescription;
    hResult = pColorSource->get_FrameDescription(&pDescription);
    if( FAILED( hResult ) ){
        std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
    }

    pDescription->get_Width(&width); // 1920
    pDescription->get_Height(&height); // 1080

    // Release
    SafeRelease(pColorSource);
    SafeRelease(pDescription);
}

void MainWindow::on_btnCapturar_clicked()
{
    _deveCapturar = true;
    capturarCor();
}

void MainWindow::on_btnParar_clicked()
{
    _deveCapturar = false;
}

void MainWindow::capturarCor()
{
    HRESULT hResult = S_OK;
    while(_deveCapturar){

        // Frame
        IColorFrame* pColorFrame = nullptr;
        hResult = pColorReader->AcquireLatestFrame( &pColorFrame );
        if( SUCCEEDED( hResult ) ){

            UINT bufferSize = 0;
            BYTE* pBuffer = nullptr;

            /*
            Raw Color Data that retrieved from Kinect v2 is YUY2 format.
            YUY2 format are aligned in order of YUYV.

            YUY2 ... 8bit, 2channels/pixel (CV_8UC2)
            BGR ... 8bit, 3channels/pixel (CV_8UC3)

            This Sample Program converts the image format to BGR
            from YUY2 using the OpenCV.

            cv::cvtColor( bufferMat, colorMat, CV_YUV2BGR_YUYV );
            */

            hResult = pColorFrame->AccessRawUnderlyingBuffer(&bufferSize, &pBuffer); // YUY2

            if(SUCCEEDED(hResult))
            {
                cv::Mat bufferMat(height, width, CV_8UC2, pBuffer);
                cv::cvtColor(bufferMat, colorMat, CV_YUV2BGR_YUYV);
                //cv::cvtColor(bufferMat, colorMat, CV_YUV2BGR_YUYV);
                //cv::cvtColor(colorMat, colorMat, CV_YUV2BGR);
                //cv::cvtColor(colorMat, colorMat, CV_BGR2RGB);
                cv::cvtColor(colorMat, colorMat, CV_YUV2BGR_YUYV);

                ui->lblCapturar->setPixmap(QPixmap::fromImage(
                    QImage((uchar*) colorMat.data, 1920, 1080,
                    //QImage::Format_RGB888).scaledToWidth(700)));
                    //QImage::Format_ARGB32).scaledToWidth(700)));
                    QImage::Format_RGB32).scaledToWidth(700)));
            }
        }
        SafeRelease(pColorFrame);
        boost::this_thread::sleep_for(boost::chrono::milliseconds{33});
    }
}

MainWindow::~MainWindow()
{
    delete ui;
    if( pSensor ){
        pSensor->Close();
    }
    SafeRelease( pSensor );
}
