#include "kinectsensor.h"

// Error Check
#define ERROR_CHECK(ret)                                      \
    if(FAILED(ret)){                                          \
    std::stringstream ss;                                     \
    ss << "failed " #ret " " << std::hex << ret << std::endl; \
    throw std::runtime_error(ss.str().c_str());               \
}

// Constructor
KinectSensor::KinectSensor()
{
    // Initialize
    initialize();
}

// Destructor
KinectSensor::~KinectSensor()
{
    // Finalize
    finalize();
}

// Get PCL Cloud
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr KinectSensor::getPCLCloud()
{
    update();
    return cloud;
}

// Initialize
void KinectSensor::initialize()
{
    // Initialize Sensor
    initializeSensor();

    // Initialize Color
    initializeColor();

    // Initialize Depth
    initializeDepth();

    // Initialize Point Cloud
    initializePointCloud();
}

// Initialize Sensor
void KinectSensor::initializeSensor()
{
    // Open Sensor
    ERROR_CHECK(GetDefaultKinectSensor(&kinect));

    ERROR_CHECK(kinect->Open());

    // Check Open
    BOOLEAN isOpen = FALSE;
    ERROR_CHECK(kinect->get_IsOpen(&isOpen));
    if(!isOpen){
        throw std::runtime_error("failed IKinectSensor::get_IsOpen( &isOpen )");
    }

    // Retrieve Coordinate Mapper
    ERROR_CHECK(kinect->get_CoordinateMapper(&coordinateMapper));
}

// Initialize Color
void KinectSensor::initializeColor()
{
    // Open Color Reader
    ComPtr<IColorFrameSource> colorFrameSource;
    ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
    ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

    // Retrieve Color Description
    ComPtr<IFrameDescription> colorFrameDescription;
    ERROR_CHECK(colorFrameSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription));
    ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth)); // 1920
    ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight)); // 1080
    ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel)); // 4

    // Allocation Color Buffer
    colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
}

// Initialize Depth
void KinectSensor::initializeDepth()
{
    // Open Depth Reader
    ComPtr<IDepthFrameSource> depthFrameSource;
    ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
    ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

    // Retrieve Depth Description
    ComPtr<IFrameDescription> depthFrameDescription;
    ERROR_CHECK(depthFrameSource->get_FrameDescription(&depthFrameDescription));
    ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth)); // 512
    ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight)); // 424
    ERROR_CHECK(depthFrameDescription->get_BytesPerPixel(&depthBytesPerPixel) ); // 2

    // Allocation Depth Buffer
    depthBuffer.resize(depthWidth * depthHeight);
}

// Initialize Point Cloud
void KinectSensor::initializePointCloud()
{
    // Create Point Cloud
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloud->width = static_cast<uint32_t>(depthWidth);
    cloud->height = static_cast<uint32_t>(depthHeight);
    cloud->points.resize(cloud->height * cloud->width);
    cloud->is_dense = false;
}

// Update Data
void KinectSensor::update()
{
    // Update Color
    updateColor();

    // Update Depth
    updateDepth();

    // Update Point Cloud
    updatePointCloud();
}

// Update Color
void KinectSensor::updateColor()
{
    // Retrieve Color Frame
    ComPtr<IColorFrame> colorFrame;
    const HRESULT ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
    if(FAILED(ret)){
        return;
    }

    // Convert Format ( YUY2 -> BGRA )
    ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra));
}

// Update Depth
void KinectSensor::updateDepth()
{
    // Retrieve Depth Frame
    ComPtr<IDepthFrame> depthFrame;
    const HRESULT ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
    if(FAILED(ret)){
        return;
    }

    // Retrieve Depth Data
    ERROR_CHECK(depthFrame->CopyFrameDataToArray(static_cast<UINT>(depthBuffer.size()), &depthBuffer[0]));
}

// Update Point Cloud
void KinectSensor::updatePointCloud()
{
    // Reset Point Cloud
    cloud->clear();

    // Convert to Point Cloud
    for(int depthY = 0; depthY < depthHeight; depthY++){
        for(int depthX = 0; depthX < depthWidth; depthX++){
            pcl::PointXYZRGBA point;

            // Retrieve Mapped Coordinates
            DepthSpacePoint depthSpacePoint = { static_cast<float>(depthX), static_cast<float>(depthY) };
            UINT16 depth = depthBuffer[depthY * depthWidth + depthX];
            ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
            ERROR_CHECK(coordinateMapper->MapDepthPointToColorSpace( depthSpacePoint, depth, &colorSpacePoint ));

            // Set Color to Point
            int colorX = static_cast<int>(colorSpacePoint.X + 0.5f);
            int colorY = static_cast<int>(colorSpacePoint.Y + 0.5f);
            if((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)){
                unsigned int colorIndex = (colorY * colorWidth + colorX) * colorBytesPerPixel;
                point.b = colorBuffer[colorIndex + 0];
                point.g = colorBuffer[colorIndex + 1];
                point.r = colorBuffer[colorIndex + 2];
                point.a = colorBuffer[colorIndex + 3];
            }

            // Retrieve Mapped Coordinates
            CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
            ERROR_CHECK(coordinateMapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint));

            // Set Depth to Point
            if((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)){
                point.x = cameraSpacePoint.X;
                point.y = cameraSpacePoint.Y;
                point.z = cameraSpacePoint.Z;
            }

            // Set Point to Point Cloud
            cloud->push_back(point);
        }
    }
}

// Finalize
void KinectSensor::finalize()
{
    // Close Sensor
    if(kinect != nullptr){
        kinect->Close();
    }
}

