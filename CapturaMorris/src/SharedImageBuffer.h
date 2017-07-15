#ifndef SHAREDIMAGEBUFFER_H
#define SHAREDIMAGEBUFFER_H

// Qt
#include <QHash>
#include <QSet>
#include <QWaitCondition>
#include <QMutex>

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

// Local
#include <Buffer.h>

using namespace cv;

class SharedImageBuffer
{
    public:
        SharedImageBuffer();
        Buffer<Mat>* getByDeviceNumber(int deviceNumber);
        void add(int deviceNumber, Buffer<Mat> *imageBuffer, bool sync=false);
        void removeByDeviceNumber(int deviceNumber);
        void sync(int deviceNumber);
        void wakeAll();
        void setSyncEnabled(bool enable);
        bool isSyncEnabledForDeviceNumber(int deviceNumber);
        bool getSyncEnabled();
        bool containsImageBufferForDeviceNumber(int deviceNumber);

    private:
        QHash<int, Buffer<Mat>*> imageBufferMap;
        QSet<int> syncSet;
        QWaitCondition wc;
        QMutex mutex;
        int nArrived;
        bool doSync;
};

#endif // SHAREDIMAGEBUFFER_H
