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
#include <boost/format.hpp>

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

private slots:
    void atualizarFrame();

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    unsigned int m_colorBytesPerPixel;
    int imgScale = 310;

    int colorWidth  = 1920;
    int colorHeight = 1080;
    int mgColorLeft = 180;
    int mgColorRight = 220;
    int colorWidthCropped  = colorWidth - (mgColorLeft + mgColorRight);

    int infraWidth  = 512;
    int infraHeight = 424;

    int depthWidth  = 512;
    int depthHeight = 424;

public Q_SLOTS:
    void abrirArquivo();
    void atualizarTela();

Q_SIGNALS:
    void abrirArquivo(QString);

private:
    Ui::MainWindow *ui;
    QString nomeArquivoAtual;

    IKinectSensor* sensorKinect;
    KinectWriter kinectWriter;

    ICoordinateMapper* mapper;
    IMultiSourceFrameReader* msFrameReader;

    INT64 timeInicial;
    INT64 ultimaContagem;
    INT64 proximaAtualizacaoTime;
    DWORD totalDeframesAtualizados;
    double frequencia;

    RGBQUAD* _colorBuffer;
    RGBQUAD* _infraBuffer;
    RGBQUAD* _depthBuffer;

    RGBQUAD* bufferLeitura;
    RGBQUAD* croppedBuffer;

    HRESULT inicializarSensorKinect();

    void FPS(QLabel *label, INT64 nTime);

    void processarCor(INT64 colorTime, const RGBQUAD* colorBuffer, int colorWidth, int colorHeight);
    void processarInfrared(INT64 infraTime, const UINT16* infraBuffer, int infraHeight, int infraWidth);
    void processarDepth(INT64 depthTime, const UINT16* depthBuffer, int depthHeight, int depthWidth, USHORT minDepthWidth, USHORT maxDepthHeight);
};

#endif // MAINWINDOW_H
