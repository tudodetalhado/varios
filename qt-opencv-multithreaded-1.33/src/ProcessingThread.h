#ifndef PROCESSINGTHREAD_H
#define PROCESSINGTHREAD_H

// Qt
#include <QtCore/QThread>
#include <QtCore/QTime>
#include <QtCore/QQueue>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// Local
#include "Structures.h"
#include "Config.h"
#include "Buffer.h"
#include "MatToQImage.h"
#include "SharedImageBuffer.h"

using namespace cv;

class ProcessingThread : public QThread
{
    Q_OBJECT

    public:
        ProcessingThread(SharedImageBuffer *sharedImageBuffer, int deviceNumber);
        QRect getCurrentROI();
        void stop();

    private:
        void updateFPS(int);
        void setROI();
        void resetROI();
        SharedImageBuffer *sharedImageBuffer;
        Mat currentFrame;
        Mat currentFrameGrayscale;
        Rect currentROI;
        QImage frame;
        QTime t;
        QQueue<int> fps;
        QMutex doStopMutex;
        QMutex processingMutex;
        Size frameSize;
        Point framePoint;
        struct ImageProcessingFlags imgProcFlags;
        struct ImageProcessingSettings imgProcSettings;
        struct ThreadStatisticsData statsData;
        volatile bool doStop;
        int processingTime;
        int fpsSum;
        int sampleNumber;
        int deviceNumber;
        bool enableFrameProcessing;

    protected:
        void run();

    private slots:
        void updateImageProcessingFlags(struct ImageProcessingFlags);
        void updateImageProcessingSettings(struct ImageProcessingSettings);
        void setROI(QRect roi);

    signals:
        void newFrame(const QImage &frame);
        void updateStatisticsInGUI(struct ThreadStatisticsData);
};

#endif // PROCESSINGTHREAD_H
