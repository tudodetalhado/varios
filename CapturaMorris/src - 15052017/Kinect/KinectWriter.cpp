#include "KinectWriter.h"
#include "Common.h"
#include <QTimer>
#include <QEventLoop>
#include <QThread>
#include <QDebug>
#include <QRect>

using namespace std;

int scale = 1;
cv::Size sz(1920/scale,1080/scale);

string now() {
  char s[1024];
  time_t t = time(NULL);
  struct tm lnow;
  localtime_s(&lnow, &t);
  sprintf_s(s, "%04d-%02d-%02d_%02d-%02d-%02d", lnow.tm_year + 1900, lnow.tm_mon + 1, lnow.tm_mday, lnow.tm_hour, lnow.tm_min, lnow.tm_sec);
  return string(s);
}

KinectWriter::KinectWriter(QObject *parent, FIFOBuffer<cv::Mat> *circularBuffer)
    : QThread(parent)
{
    this->_circularBuffer = circularBuffer;
    doStop = false;
    _deveProcessar = false;
}

KinectWriter::~KinectWriter()
{
    _videoWriter.release();
}

void KinectWriter::iniciar()
{
    mutex.lock();
    _deveProcessar = true;
    //_videoWriter = cv::VideoWriter(now()+".avi",CV_FOURCC_MACRO('X','V','I','D'), 30.0, sz);
    //_videoWriter = cv::VideoWriter(now()+".mp4",CV_FOURCC_MACRO('M','P','4','V'), 30.0, sz);
    _videoWriter = cv::VideoWriter(now()+".mp4",CV_FOURCC_MACRO('F','M','P','4'), 30.0, sz);
    mutex.unlock();
    start((QThread::Priority)QThread::IdlePriority);
    //emit iniciarSignal();
}

void KinectWriter::finalizar()
{
    mutex.lock();
    _deveProcessar = false;
    _videoWriter.release();
    mutex.unlock();
}

void KinectWriter::stop()
{
    mutex.lock();
    _deveProcessar = false;
    _videoWriter.release();
    mutex.unlock();
    QMutexLocker locker(&doStopMutex);
    doStop=true;
}

int contador = 1;
void KinectWriter::run()
{
    while(1)
    {
        // Stop thread if doStop=TRUE //
        doStopMutex.lock();
        if(doStop)
        {
            doStop = false;
            doStopMutex.unlock();
            break;
        }
        doStopMutex.unlock();


        //queueProtected.lock();
        // Get frame from queue, store in currentFrame, set ROI
        //Rect currentROI = Rect(0, 0, 1920, 1080);

        //Buffer<Mat> *buffer = sharedImageBuffer->getByDeviceNumber(1);

        //if (buffer->size() > 0){
        //    currentFrame = Mat(buffer->get().clone()); //, currentROI);
        //    _videoWriter << currentFrame;
        //}


        //if (_queue.size() > 0)
        //{
            //usedSlots->acquire();

            // Take item from queue
           // _queueProtected.lock();
            //currentFrame = _queue[0];
            // _circularBuffer->pop().copyTo(_currentFrame);
           // _queueProtected.unlock();

            // Release semaphores
            //freeSlots->release();

            //_videoWriter << _currentFrame;
        //}

        // Example of how to grab a frame from another stream (where Device Number=1)
        // Note: This requires stream synchronization to be ENABLED (in the Options menu of MainWindow) and frame processing for the stream you are grabbing FROM to be DISABLED.
        /*
        if(sharedImageBuffer->containsImageBufferForDeviceNumber(1))
        {
            // Grab frame from another stream (connected to camera with Device Number=1)
            Mat frameFromAnotherStream = Mat(sharedImageBuffer->getByDeviceNumber(1)->getFrame(), currentROI);
            // Linear blend images together using OpenCV and save the result to currentFrame. Note: beta=1-alpha
            addWeighted(frameFromAnotherStream, 0.5, currentFrame, 0.5, 0.0, currentFrame);
        }
        */


        //QEventLoop loop;
        //QTimer::singleShot(100, &loop, SLOT(quit()));
        //loop.exec();



        //emit processadoSignal(QString::number(contador++));

        //queueProtected.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(22));


    }
}

void KinectWriter::gravarFrame(cv::Mat frm)
{
    //_videoWriter << frm;
    //QEventLoop loop;
    //QTimer::singleShot(100, &loop, SLOT(quit()));
    //loop.exec();
    //emit processadoSignal(QString::number(contador++));
}

//void KinectWriter::processarSlot()
//{
//    _videoWriter << _frm;
//    QEventLoop loop;
//    QTimer::singleShot(100, &loop, SLOT(quit()));
//    loop.exec();
//    //emit processadoSignal(QString::number(contador++));
//    _frm = NULL;

////    int contador = 1;
////    while(_deveProcessar)
////    {
////        //cv::resize(kinect.rgbImage, frm, sz, 0, 0);
////        //cv::cvtColor(frm, frm, CV_BGRA2BGR);
////        if (!_frm)
////        {
////            _videoWriter << _frm;
////            QEventLoop loop;
////            QTimer::singleShot(100, &loop, SLOT(quit()));
////            loop.exec();
////            emit processadoSignal(QString::number(contador++));
////            _frm = NULL;
////        }
////    }
//    emit concluidoSignal();
//}
