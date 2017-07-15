#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H

// Qt
#include <QtCore/QTime>
#include <QtCore/QThread>

// OpenCV
#include <opencv2/highgui/highgui.hpp>

// Local
#include "SharedImageBuffer.h"
#include "Config.h"
#include "Structures.h"

using namespace cv;

class ImageBuffer;

class CaptureThread : public QThread
{
    Q_OBJECT

    public:
        CaptureThread(SharedImageBuffer *sharedImageBuffer,
          int deviceNumber, bool dropFrameIfBufferFull, int width, int height);
        void stop();
        bool connectToCamera();
        bool disconnectCamera();
        bool isCameraConnected();
        int getInputSourceWidth();
        int getInputSourceHeight();

    private:
        void updateFPS(int);
        SharedImageBuffer *sharedImageBuffer;
        VideoCapture cap;
        Mat grabbedFrame;
        QTime t;
        QMutex doStopMutex;
        QQueue<int> fps;
        struct ThreadStatisticsData statsData;
        volatile bool doStop;
        int captureTime;
        int sampleNumber;
        int fpsSum;
        bool dropFrameIfBufferFull;
        int deviceNumber;
        int width;
        int height;

    protected:
        void run();

    signals:
        void updateStatisticsInGUI(struct ThreadStatisticsData);
};

#endif // CAPTURETHREAD_H
