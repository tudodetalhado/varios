#ifndef __APP__
#define __APP__

#include <Windows.h>
#include <Kinect.h>
// Quote from MSDN Forums - Joint Smoothing code, and Partial Modification
// KinectJointFilter is: Copyright (c) Microsoft Corporation. All rights reserved.
// https://social.msdn.microsoft.com/Forums/en-US/045b058a-ae3a-4d01-beb6-b756631b4b42
#include "KinectJointFilter.h"
#include <opencv2/opencv.hpp>

#include <vector>
#include <array>

#include <wrl/client.h>
using namespace Microsoft::WRL;

class Kinect
{
private:
    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Coordinate Mapper
    ComPtr<ICoordinateMapper> coordinateMapper;

    // Reader
    ComPtr<IColorFrameReader> colorFrameReader;
    ComPtr<IBodyFrameReader> bodyFrameReader;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;
    cv::Mat colorMat;

    // Body Buffer
    std::array<IBody*, BODY_COUNT> bodies = { nullptr };
    std::array<cv::Vec3b, BODY_COUNT> colors;

    // Smoothing Filter
    std::array<Sample::FilterDoubleExponential, BODY_COUNT> filters;

public:
    // Constructor
    Kinect();

    // Destructor
    ~Kinect();

    // Processing
    void run();

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    inline void initializeSensor();

    // Initialize Color
    inline void initializeColor();

    // Initialize Body
    inline void initializeBody();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Color
    inline void updateColor();

    // Update Body
    inline void updateBody();

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw Body
    inline void drawBody();

    // Draw Smooth
    inline void drawSmooth();

    // Draw Circle
    inline void drawEllipse( cv::Mat& image, const Joint& joint, const int radius, const cv::Vec3b& color, const int thickness = -1 );

    // Draw Hand State
    inline void drawHandState( cv::Mat& image, const Joint& joint, HandState handState, TrackingConfidence handConfidence );

    // Show Data
    void show();

    // Show Body
    inline void showBody();
};

#endif // __APP__