//#include "stdafx.h"
#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>
#include <pcl/visualization/cloud_viewer.h>


template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
    if( pInterfaceToRelease != NULL ){
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

int main(int argc, char* argv[])
{
    UINT nBufferSize = 0;


    // Create Sensor Instance
    IKinectSensor* pSensor;
    HRESULT hResult = S_OK;
    hResult = GetDefaultKinectSensor( &pSensor );
    if( FAILED( hResult ) ){
        std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
        return -1;
    }

    // Open Sensor
    hResult = pSensor->Open();
    if( FAILED( hResult ) ){
        std::cerr << "Error : IKinectSensor::Open()" << std::endl;
        return -1;
    }

    // Retrieved Coordinate Mapper
    ICoordinateMapper* pCoordinateMapper;
    hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper );
    if( FAILED( hResult ) ){
        std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
        return -1;
    }

    // Retrieved Color Frame Source
    IColorFrameSource* pColorSource;
    hResult = pSensor->get_ColorFrameSource( &pColorSource );
    if( FAILED( hResult ) ){
        std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
        return -1;
    }

    // Retrieved Depth Frame Source
    IDepthFrameSource* pDepthSource;
    hResult = pSensor->get_DepthFrameSource( &pDepthSource );
    if( FAILED( hResult ) ){
        std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
        return -1;
    }

    // Open Color Frame Reader
    IColorFrameReader* pColorReader;
    hResult = pColorSource->OpenReader( &pColorReader );
    if( FAILED( hResult ) ){
        std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
        return -1;
    }

    // Open Depth Frame Reader
    IDepthFrameReader* pDepthReader;
    hResult = pDepthSource->OpenReader( &pDepthReader );
    if( FAILED( hResult ) ){
        std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
        return -1;
    }

    // Retrieved Color Frame Size
    IFrameDescription* pColorDescription;
    hResult = pColorSource->get_FrameDescription( &pColorDescription );
    if( FAILED( hResult ) ){
        std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
        return -1;
    }
    int colorWidth = 0;
    int colorHeight = 0;
    pColorDescription->get_Width(&colorWidth); // 1920
    pColorDescription->get_Height(&colorHeight); // 1080

    // To Reserve Color Frame Buffer
    RGBQUAD *colorBuffer = NULL;

    // Retrieved Depth Frame Size
    IFrameDescription* pDepthDescription;
    hResult = pDepthSource->get_FrameDescription( &pDepthDescription );
    if( FAILED( hResult ) ){
        std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
        return -1;
    }
    int depthWidth = 0;
    int depthHeight = 0;
    pDepthDescription->get_Width( &depthWidth ); // 512
    pDepthDescription->get_Height( &depthHeight ); // 424

    // To Reserve Depth Frame Buffer
    //std::vector<RGBQUAD*> depthBuffer( depthWidth * depthHeight );
    UINT16 *depthBuffer = NULL;

    // Create Cloud Viewer
    pcl::visualization::CloudViewer viewer( "Point Cloud Viewer" );

    while( !viewer.wasStopped() ){
        // Acquire Latest Color Frame
        IColorFrame* pColorFrame = nullptr;
        hResult = pColorReader->AcquireLatestFrame( &pColorFrame );
        if( SUCCEEDED( hResult ) ){
            // Retrieved Color Data
            //hResult = pColorFrame->CopyConvertedFrameDataToArray( colorBuffer.size() * sizeof( RGBQUAD ), reinterpret_cast<BYTE*>( &colorBuffer[0] ), ColorImageFormat::ColorImageFormat_Bgra );
            //hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
            hResult = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&colorBuffer));

            if( FAILED( hResult ) ){
                std::cerr << "Error : IColorFrame::CopyConvertedFrameDataToArray()" << std::endl;
            }
        }
        SafeRelease( pColorFrame );

        // Acquire Latest Depth Frame
        IDepthFrame* pDepthFrame = nullptr;
        hResult = pDepthReader->AcquireLatestFrame( &pDepthFrame );
        if( SUCCEEDED( hResult ) ){
            // Retrieved Depth Data
            //hResult = pDepthFrame->CopyFrameDataToArray( depthBuffer.size(), &depthBuffer[0] );
            //hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
            hResult = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &depthBuffer);

            if( FAILED( hResult ) ){
                std::cerr << "Error : IDepthFrame::CopyFrameDataToArray()" << std::endl;
            }
        }
        SafeRelease( pDepthFrame );

        // Create Point Cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
        pointcloud->width = static_cast<uint32_t>( depthWidth );
        pointcloud->height = static_cast<uint32_t>( depthHeight );
        pointcloud->is_dense = false;

        for( int y = 0; y < depthHeight; y++ ){
            for( int x = 0; x < depthWidth; x++ ){
                pcl::PointXYZRGB point;

                DepthSpacePoint depthSpacePoint = { static_cast<float>( x ), static_cast<float>( y ) };
                UINT16 depth = depthBuffer[y * depthWidth + x];

                // Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
                ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
                pCoordinateMapper->MapDepthPointToColorSpace( depthSpacePoint, depth, &colorSpacePoint );
                int colorX = static_cast<int>( std::floor( colorSpacePoint.X + 0.5f ) );
                int colorY = static_cast<int>( std::floor( colorSpacePoint.Y + 0.5f ) );
                if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                    RGBQUAD color = colorBuffer[colorY * colorWidth + colorX];
                    point.b = color.rgbBlue;
                    point.g = color.rgbGreen;
                    point.r = color.rgbRed;
                }

                // Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
                CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
                pCoordinateMapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint );
                if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                    point.x = cameraSpacePoint.X;
                    point.y = cameraSpacePoint.Y;
                    point.z = cameraSpacePoint.Z;
                }

                pointcloud->push_back( point );
            }
        }

        // Show Point Cloud on Cloud Viewer
        viewer.showCloud( pointcloud );

        // Input Key ( Exit ESC key )
        if( GetKeyState( VK_ESCAPE ) < 0 ){
            break;
        }
    }

    // End Processing
    SafeRelease( pColorSource );
    SafeRelease( pDepthSource );
    SafeRelease( pColorReader );
    SafeRelease( pDepthReader );
    SafeRelease( pColorDescription );
    SafeRelease( pDepthDescription );
    SafeRelease( pCoordinateMapper );
    if( pSensor ){
        pSensor->Close();
    }
    SafeRelease( pSensor );

    return 0;
}
