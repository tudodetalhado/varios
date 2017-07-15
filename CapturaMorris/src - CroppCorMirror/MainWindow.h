#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <chrono>
#include <strsafe.h>
#include <fstream>
#include <thread>
#include <mutex>
#include "stdafx.h"
#include "Utils.h"
#include "KinectSensor.h"
#include "KinectWriter.h"
#include "KinectReader.h"
#include <QThread>
#include <QLabel>
#include <QTimer>
#include <boost/format.hpp>
#include "Cronometro.h"

// InfraredSourceValueMaximum is the highest value that can be returned in the InfraredFrame.
// It is cast to a float for readability in the visualization code.
#define InfraredSourceValueMaximum static_cast<float>(USHRT_MAX)

// The InfraredOutputValueMinimum value is used to set the lower limit, post processing, of the
// infrared data that we will render.
// Increasing or decreasing this value sets a brightness "wall" either closer or further away.
#define InfraredOutputValueMinimum 0.01f

// The InfraredOutputValueMaximum value is the upper limit, post processing, of the
// infrared data that we will render.
#define InfraredOutputValueMaximum 1.0f

// The InfraredSceneValueAverage value specifies the average infrared value of the scene.
// This value was selected by analyzing the average pixel intensity for a given scene.
// Depending on the visualization requirements for a given application, this value can be
// hard coded, as was done here, or calculated by averaging the intensity for each pixel prior
// to rendering.
#define InfraredSceneValueAverage 0.08f

/// The InfraredSceneStandardDeviations value specifies the number of standard deviations
/// to apply to InfraredSceneValueAverage. This value was selected by analyzing data
/// from a given scene.
/// Depending on the visualization requirements for a given application, this value can be
/// hard coded, as was done here, or calculated at runtime.
#define InfraredSceneStandardDeviations 3.0f

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public Q_SLOTS:
    void abrirTrack();
    void atualizarTela();
    void gravarTrack();
    void setPararGravacaoAuto();
    void atualizarLCD();
    void atualizarFrame();
    void alterarStatus();

//Q_SIGNALS:
//    void abrirTrack(QString);


private:
    Ui::MainWindow *ui;
    QString nomeArquivoAtual;
    Cronometro _cron;
    QTimer _timer;

    IKinectSensor* sensorKinect;
    KinectWriter kinectWriter;

    ICoordinateMapper* mapper;
    IMultiSourceFrameReader* msFrameReader;

    INT64 timeInicial;
    INT64 ultimaContagem;
    INT64 proximaAtualizacaoTime;
    DWORD totalDeframesAtualizados;
    double frequencia;

    unsigned int m_colorBytesPerPixel;
    int imgScale = 424;

    bool _deveProcessar = true;
    bool _deveGravar = false; 
    bool _pararGravacaoAuto  = true;

    INT64 _colorTime;
    INT64 _infraTime;
    INT64 _depthTime;

    const int rgbQuadSize = sizeof(RGBQUAD);
    const int _1920 = 1920;
    const int _1080 = 1080;
    const int _512  = 512;
    const int _424  = 424;

    int mgColorLeft = 180;
    int mgColorRight = 220;
    int colorWidthCropped  = _1920 - (mgColorLeft + mgColorRight);

    RGBQUAD* _1920x1080_rgb_buffer;
    RGBQUAD* _1080x1080_rgb_buffer;
    UINT16*  _512x424_int_buffer;
    RGBQUAD* _512x424_rgb_buffer;
    RGBQUAD* _424x424_rgb_buffer;
    UINT16*  _424x424_int_buffer;

    RGBQUAD* _depthBuffer;
    RGBQUAD* _tempDepthBuffer;

    RGBQUAD* _croppedBuffer;

    RGBQUAD* bufferLeitura;



    HRESULT inicializarSensorKinect();

    void FPS(QLabel *label, INT64 time);

    void definirTela();
    void processarCor();
    void processarInfrared();
    void processarDepth(INT64 depthTime, const UINT16* depthBuffer, int _424, int _512, USHORT minDepthWidth, USHORT maxDepthHeight);
};

#endif // MAINWINDOW_H
