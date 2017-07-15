#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "Utils.h"
#include "stdafx.h"
#include "Cronometro.h"

#include <mutex>
#include <thread>
#include <strsafe.h>
#include <fstream>

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <QActionGroup>
#include <QMouseEvent>
#include <QPushButton>
#include <QSemaphore>
#include <QThread>
#include <QLabel>
#include <QTimer>
#include <chrono>

#include "Kinect/KinectSensor.h"
#include "Kinect/KinectWriter.h"
#include "Kinect/KinectReader.h"

#include <boost/format.hpp>
#include <opencv2/opencv.hpp>
#include <ctkRangeSlider.h>
#include <ArenaView.h>
#include <CenaArena.h>
#include <ItemArena.h>
#include <FIFOBuffer.h>


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
    static const int _frmWidth_1920  = 1920;
    static const int _frmHeight_1080 = 1080;
    static const int _frmWidth_640   = 640;
    static const int _frmHeight_530  = 530;

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void alternarPushButton(bool ativar);
    
    void inciarKinectWriter();
    
    void finalizarKinectWriter();
    
public Q_SLOTS:
    void deveGravarVideoSlot();


    void onKinectCallback(RGBQUAD *buffer, KinectTime *time);
    void onKinectWriterCallback();

    void desmarcarFerramentas();
    void salvarArena();
    void atualizarTela();
    void abrirTrack();
    void tocarTrack(QString str);
    void capturar();

    void setPararGravacaoAuto();
    void atualizarLCD();
    void atualizarDisplayPlay();
    //void lerDadosSensorKinect();
    void alterarStatus();
    void dimensionarCirculo(int valor);
    void posicionarX(int valor);
    void posicionarY(int valor);
    void ajustarContraste(int valor);
    void inverterContraste();
    void ajustarMatiz(int,int);
    void ajustarSaturacao(int,int);
    void ajustarValor(int, int);
    void ajustarContraste(int, int);
    void ajustarVerde(int,int);
    void ajustarVermelho(int, int);
    void ajustarAzul(int, int);
    void toggled();
    //void duploCliqueArenaView();

private:
    Ui::MainWindow *ui;
    //QGraphicsScene *cena;
    QGraphicsEllipseItem *ellipse;
    QGraphicsRectItem *rectangle;
    ItemArena *arena;

    QThread *_writerThread;
    KinectSensor *sensorKinect;
    KinectWriter *_kinectWriter;
    FIFOBuffer<cv::Mat> *_circularBuffer;

    //ArenaCena* arenaCena;
    //QGraphicsView* view;
    QAction* lineAction;
    QAction* selectAction;
    QActionGroup *actionGroup;
    //QToolBar* drawingToolBar;

    void createActions();
    void createConnections();

    QString nomeArquivoAtual;
    Cronometro _cron;
    Cronometro _cronPlay;
    Cronometro _cronCalc;
    Cronometro _cronAuto;
    QTimer _timer;
    QTimer _timerPlay;

    void showImage(cv::Mat img, QLabel *lbl);

    const std::string hsvWindow = "HSV Image";


    ICoordinateMapper* mapper;
    IMultiSourceFrameReader* msFrameReader;

    INT64 timeInicial;
    INT64 ultimaContagem;
    INT64 proximaAtualizacaoTime;
    DWORD totalDeframesAtualizados;
    double frequencia;

    int contraste = 127;

    bool _deveProcessar = true;
    bool _deveGravarVideo = false;
    bool _pararGravacaoAuto  = true;
    bool _invertido = false;
    bool deveQuantificar = false;

    INT64 _colorTime;
    INT64 _infraTime;
    INT64 _depthTime;
    INT64 _playTime;
    int _raioArena  = 175;  //180
    int _raioPlataforma  = 20;  //180
    int _eixoXArena = 250;  //256
    int _eixoYArena = 210;  //212

    int _x, _y;
    float distancia_em_cm = 0;

    int _minMatiz   = 0;
    int _minSaturar = 0;
    int _minBrilho  = 0;
    int _minContraste = 0;
    int _minVerde     = 0;
    int _minVermelho  = 0;
    int _minAzul      = 0;

    int _maxMatiz   = 255;
    int _maxSaturar = 255;
    int _maxBrilho  = 255;
    int _maxContraste  = 255;
    int _maxVerde   = 255;
    int _maxVermelho = 255;
    int _maxAzul  = 255;

    const USHORT _minDepth = 500;
    const USHORT _maxDepth = 4500;

    //cor
    RGBQUAD* _1920x1080_cor_buffer;
    RGBQUAD* _1304x1080_cor_buffer;
    RGBQUAD* _512x424_cor_buffer;
    RGBQUAD* _640x530_cor_buffer;
    std::vector<BYTE> colorBuffer;
    cv::Mat frameColor;

    //infra
    UINT16*  _512x424_infraInt_buffer;
    RGBQUAD* _512x424_infraCor_buffer;

    //depth
    UINT16*  _512x424_depthInt_buffer;
    RGBQUAD* _512x424_depthCor_buffer;
    std::vector<UINT16> depthBuffer;

    HRESULT inicializarSensorKinect();

    void FPS(QLabel *label, INT64 time);

//    void resizeEvent(QResizeEvent *);
//    void showEvent(QShowEvent *);
//    void fitView();
    void definirTela();
    void processarCor();
    void processarInfrared();
    void processarDepth();
    void mousePressEvent(QMouseEvent *event);
    void adicionarRegistro(cv::Mat frm);
};

#endif // MAINWINDOW_H
