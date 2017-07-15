#ifndef KINECTSENSOR_H
#define KINECTSENSOR_H

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <wrl/client.h>
using namespace Microsoft::WRL;

class KinectSensor
{
private:
    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Coordinate Mapper
    ComPtr<ICoordinateMapper> coordinateMapper;

    // Reader
    ComPtr<IColorFrameReader> colorFrameReader;
    ComPtr<IDepthFrameReader> depthFrameReader;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;

    // Depth Buffer
    std::vector<UINT16> depthBuffer;
    int depthWidth;
    int depthHeight;
    unsigned int depthBytesPerPixel;

    // PCL
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

public:
    // Constructor
    KinectSensor();

    // Destructor
    ~KinectSensor();

    // PCL Point Cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPCLCloud();

private:

    // Initialize
    void initialize();

    // Initialize Sensor
    void initializeSensor();

    // Initialize Color
    void initializeColor();

    // Initialize Depth
    void initializeDepth();

    // Initialize Point Cloud
    void initializePointCloud();

    // Update Data
    void update();

    // Update Color
    void updateColor();

    // Update Depth
    void updateDepth();

    // Update Point Cloud
    void updatePointCloud();

    // Finalize
    void finalize();
};

#endif // KINECTSENSOR_H
