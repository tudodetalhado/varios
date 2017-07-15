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

    int cInfraredWidth  = 512;
    int cInfraredHeight = 424;
    int cColorWidth  = 1920;
    int cColorHeight = 1080;
    int cDepthWidth  = 512;
    int cDepthHeight = 424;

private:
    Ui::MainWindow *ui;

    IKinectSensor* m_pKinectSensor;

    IInfraredFrameReader* m_pInfraredFrameReader;
    IColorFrameReader* m_pColorFrameReader;
    IDepthFrameReader* m_pDepthFrameReader;

    RGBQUAD* m_pInfraredRGBX;
    RGBQUAD* m_pColorRGBX;
    RGBQUAD* m_pDepthRGBX;

    HRESULT InitializeDefaultSensor();
    void ProcessInfrared(INT64 nTime, const UINT16* pBuffer, int nHeight, int nWidth);
    void ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight);
    void ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nHeight, int nWidth, USHORT nMinDepth, USHORT nMaxDepth);
};

#endif // MAINWINDOW_H
