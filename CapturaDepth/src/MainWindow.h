#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "Kinect.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

private slots:
    void atualizarDados();

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    UINT16 *pDepth;
    int nColorFrameHeight, nColorFrameWidth;
    int nDepthFrameHeight, nDepthFrameWidth;
    const int cDepthWidth  = 512;
    const int cDepthHeight = 424;

private:
    Ui::MainWindow *ui;
    uint64_t m_nStartTime;
    uint64_t m_nLastCounter;
    double m_fFreq;
    uint64_t m_nNextStatusTime;
    IKinectSensor* m_pKinectSensor;
    IDepthFrameReader* m_pDepthFrameReader;
    ICoordinateMapper* m_pCoordinateMapper;

    RGBQUAD* m_pDepthRGBX;
    HRESULT inicializarSensor();
    void ProcessDepth(INT64 nTime, const UINT16* pBuffer,
         int nHeight, int nWidth, USHORT nMinDepth, USHORT nMaxDepth);

};

#endif // MAINWINDOW_H
