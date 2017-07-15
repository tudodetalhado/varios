#pragma once

#include "stdafx.h"
#include <strsafe.h>
#include <fstream>
#include <mutex>
#include <thread>
#include "Utils.h"

#include <windows.h>
#include <ppl.h>
#include <iostream>
#include <random>
#include <boost/format.hpp>
#include <Boost/scoped_array.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>

#include <QThread>
#include <QMutex>
#include <QObject>
#include <QLabel>

#include "Kinect.h"
#include "KinectTime.h"

class KinectSensor : public QThread
{
    Q_OBJECT
    static const int _frmColorWidth  = 1920;
    static const int _frmColorHeight = 1080;

signals:
    void kinectColorCallback(byte*);

public slots:
    void pararDeGravar();

public:
    explicit KinectSensor(bool deveGravarVideo, int milliseconds = 33);
    void run();

    bool _deveCapturar = true;

    ~KinectSensor();

    bool inicializar();

    //void MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints);
    //void MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints);
    //void MapDepthFrameToColorSpace(Point2f *pColorSpacePoints);
    //void MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints);

    UINT16* getDepthFrame();
    //RGBQUAD* getColorFrame(IMultiSourceFrame* multiFrame);
    UINT16* getInfraredFrame();
    void processarInfrared(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight);

    bool inicializado;
    int depthFrameHeight, depthFrameWidth;
    USHORT nDepthMinReliableDistance;
    USHORT nDepthMaxDistance;

    UINT16  *bufferDeProfundidade;
    //BYTE    *_bufferColorKinect;
    byte    *_bufferColorKinect;
    RGBQUAD *_1920x1080_cor_buffer;
    RGBQUAD *_1304x1080_cor_buffer;
    RGBQUAD *_640x530_cor_buffer;
    INT64 _colorTime;
    void stop();


private:
    int milliseconds;
    IKinectSensor* kinectSensor;
    ICoordinateMapper* pCoordinateMapper;
    IMultiSourceFrameReader* msFrameReader;
    cv::VideoWriter _videoWriter;
    void gravarVideo(cv::Mat frm);
    UINT _colorBufferSize;

    QMutex doStopMutex;
    QMutex processingMutex;
    QMutex _bufferProtect;
    volatile bool doStop;
    bool _deveGravar = true;
    bool capturarFrame();

    void frameCropping();
    void frameDownsampling();
    void frameMirroring();


};
