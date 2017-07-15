#ifndef KINECTWRITER_H
#define KINECTWRITER_H

#include <thread>
#include <QThread>
#include <QMutex>
#include <QObject>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include "FIFOBuffer.h"

class KinectWriter : public QThread
{
    Q_OBJECT

public:
    explicit KinectWriter(QObject *parent = 0,
          FIFOBuffer<cv::Mat> *_circularBuffer = NULL);
    void run();

    ~KinectWriter();
    void iniciar();
    void finalizar();
    void gravarFrame(cv::Mat frm);
    void stop();

private:
    bool _deveProcessar;
    cv::VideoWriter _videoWriter;
    cv::Mat _frm;
    QMutex mutex;

    FIFOBuffer<cv::Mat> *_circularBuffer;
    QMutex doStopMutex;
    QMutex _queueProtected;
    volatile bool doStop;
    cv::Mat _currentFrame;

signals:
    void iniciarSignal();
    void processadoSignal(const QString &value);
    void concluidoSignal();

//public slots:
//    void gravarFrame(cv::Mat frm);

};

#endif // KINECTWRITER_H
