#pragma once

#include <chrono>
#include <thread>
#include <QThread>
#include <QObject>
#include <QLabel>
#include "stdafx.h"
#include "Kinect.h"
#include "KinectTime.h"
#include "Utils.h"
#include <opencv2/opencv.hpp>
#include "NtKinect.h"

class KinectSensor : public QThread
{
    Q_OBJECT

signals:
    void kinectCallback(RGBQUAD*, KinectTime*);

public:
    explicit KinectSensor(QObject *parent = 0, int milliseconds = 33);
    void run();
    bool _deveCapturar = true;

    ~KinectSensor();

    bool inicializar();

    void MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints);
    void MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints);
    void MapDepthFrameToColorSpace(Point2f *pColorSpacePoints);
    void MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints);

    UINT16* getDepthFrame();
    RGBQUAD* getColorFrame(IMultiSourceFrame* multiFrame);
    UINT16* getInfraredFrame();
    void processarInfrared(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight);

    bool inicializado;

    int colorFrameWidth = 1920, colorFrameHeight = 1080;
    int depthFrameHeight, depthFrameWidth;
    USHORT nDepthMinReliableDistance;
    USHORT nDepthMaxDistance;

    UINT16 *bufferDeProfundidade;
    RGBQUAD *bufferDeCor;

    RGBQUAD *_1920x1080_cor_buffer;
    RGBQUAD *_1304x1080_cor_buffer;
    RGBQUAD *_640x530_cor_buffer;

    cv::Mat _frmColor;
    BYTE *bytesBuffer;

    KinectTime *_time;
    INT64 _colorTime;
    UINT _colorBufferSize;

private:	
    int milliseconds;
    IKinectSensor* kinectSensor;
    ICoordinateMapper* pCoordinateMapper;
    IMultiSourceFrameReader* msFrameReader;
    bool capturarFrame();
    cv::Mat capturarOpenCVFrame();
};

