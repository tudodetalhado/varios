#ifndef KINECTWRITER_H
#define KINECTWRITER_H

#include <QObject>
#include <QMutex>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

class KinectWriter : public QObject
{
    Q_OBJECT

public:
    explicit KinectWriter(QObject *parent = 0);
    ~KinectWriter();
    void iniciar();
    void finalizar();
    void gravarFrame(cv::Mat frm);

private:
    bool _deveProcessar;
    cv::VideoWriter _videoWriter;
    cv::Mat _frm;
    QMutex mutex;

signals:
    void iniciarSignal();
    void processadoSignal(const QString &value);
    void concluidoSignal();

//public slots:
//    void gravarFrame(cv::Mat frm);

};

#endif // KINECTWRITER_H
