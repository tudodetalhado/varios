#include "KinectWriter.h"
#include <QTimer>
#include <QEventLoop>
#include <QThread>
#include <QDebug>

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

KinectWriter::KinectWriter(QObject *parent) :
    QObject(parent)
{
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
    emit iniciarSignal();
}

void KinectWriter::finalizar()
{
    mutex.lock();
    _deveProcessar = false;
    _videoWriter.release();
    mutex.unlock();
}

int contador = 1;
void KinectWriter::gravarFrame(cv::Mat frm)
{
    _videoWriter << frm;
    //QEventLoop loop;
    //QTimer::singleShot(100, &loop, SLOT(quit()));
    //loop.exec();
    emit processadoSignal(QString::number(contador++));
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
