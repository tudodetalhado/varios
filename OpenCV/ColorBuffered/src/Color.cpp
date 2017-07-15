#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>


template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
    if( pInterfaceToRelease != NULL ){
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

int _tmain( int argc, _TCHAR* argv[] )
{
    cv::setUseOptimized( true );

    // Sensor
    IKinectSensor* pSensor;
    HRESULT hResult = S_OK;
    hResult = GetDefaultKinectSensor( &pSensor );
    if( FAILED( hResult ) ){
        std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
        return -1;
    }

    hResult = pSensor->Open();
    if( FAILED( hResult ) ){
        std::cerr << "Error : IKinectSensor::Open()" << std::endl;
        return -1;
    }

    // Source
    IColorFrameSource* pColorSource;
    hResult = pSensor->get_ColorFrameSource( &pColorSource );
    if( FAILED( hResult ) ){
        std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
        return -1;
    }

    // Reader
    IColorFrameReader* pColorReader;
    hResult = pColorSource->OpenReader( &pColorReader );
    if( FAILED( hResult ) ){
        std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
        return -1;
    }

    // Description
    IFrameDescription* pDescription;
    hResult = pColorSource->get_FrameDescription( &pDescription );
    if( FAILED( hResult ) ){
        std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
        return -1;
    }

    int width = 0;
    int height = 0;
    pDescription->get_Width( &width ); // 1920
    pDescription->get_Height( &height ); // 1080

    cv::Mat colorMat( height, width, CV_8UC3 );
    cv::namedWindow( "Color" );

    while( 1 ){
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
            This Sample Program converts the image format to BGR from YUY2 using the OpenCV.

            cv::cvtColor( bufferMat, colorMat, CV_YUV2BGR_YUYV );
            */

            hResult = pColorFrame->AccessRawUnderlyingBuffer( &bufferSize, &pBuffer ); // YUY2
            if( SUCCEEDED( hResult ) ){
                cv::Mat bufferMat( height, width, CV_8UC2, pBuffer );
                cv::cvtColor( bufferMat, colorMat, CV_YUV2BGR_YUYV );
            }
        }
        SafeRelease( pColorFrame );

        // Draw
        cv::imshow( "Color", colorMat );
        if( cv::waitKey( 30 ) == VK_ESCAPE ){
            break;
        }
    }

    // Release
    SafeRelease( pColorSource );
    SafeRelease( pColorReader );
    SafeRelease( pDescription );
    if( pSensor ){
        pSensor->Close();
    }
    SafeRelease( pSensor );
    cv::destroyAllWindows();

    return 0;
}
