#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QFileDialog>
#include <QDebug>
#include <QMessageBox>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <videoInput.h>
#include <QTimer>
#include <iostream>

QString dirOpen;
IplImage * imageRead;
CvCapture* capture;
int READCOUNT;
int FPS_open,numFrames,frameW,frameH;
QTimer * timerOpen;

READCOUNT = 0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    timerOpen = new QTimer(this);
    connect(timerOpen,SIGNAL(timeout()),this,SLOT(Read()));

    SliderFrameOn = true;

    SliderSecOn   = true;

    connect(ui->SliderFrame,SIGNAL(sliderPressed()),this,SLOT(SliderFramePress()));
    connect(ui->SliderFrame,SIGNAL(sliderReleased()),this,SLOT(SliderFrameRelease()));
    connect(ui->SliderFrame,SIGNAL(sliderMoved(int)),this,SLOT(SliderFrameMove(int)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_OpenFile_clicked()
{

dirOpen = QFileDialog::getOpenFileName(this, tr("Save File"),"/home/",tr("Videos (*.avi)"));

    if(dirOpen.isEmpty())
    {
        QMessageBox msgBox;
        msgBox.setText("open file");
        msgBox.exec();
        emit on_OpenFile_clicked();
    }else
    {
        ui->Play->setEnabled(true);
    }

}

void MainWindow::on_Play_clicked()
{
    ui->OpenFile->setEnabled(false);
    ui->Pause->setEnabled(true);
    ui->Stop->setEnabled(true);

    capture = cvCaptureFromAVI(dirOpen.toStdString().c_str());

    frameH    = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
    frameW    = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
    FPS_open  = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FPS); // frame pes secund
    numFrames = (int) cvGetCaptureProperty(capture,  CV_CAP_PROP_FRAME_COUNT); // count frames

    imageRead =  cvCreateImage(cvSize(frameW,frameH),IPL_DEPTH_8U,3);

    ui->SliderFrame->setMaximum(numFrames);
    ui->SliderSec->setMaximum( (int) (numFrames / FPS_open) );

    if(!cvGrabFrame(capture)){
        QMessageBox msgBox;
        msgBox.setText("Error");
        msgBox.exec();
    }else
    {
        ui->SliderFrame->setEnabled(true);
        READCOUNT = 0;
        timerOpen->start( (int) (1000/FPS_open) );
    }
}

void MainWindow::on_Pause_clicked()
{

    ui->Play->setEnabled(true);
    ui->Pause->setEnabled(false);
    timerOpen->stop();
}

void MainWindow::on_Stop_clicked()
{
    ui->Stop->setEnabled(false);
    ui->Play->setEnabled(false);
    ui->Pause->setEnabled(false);
    ui->OpenFile->setEnabled(true);
    dirOpen = "";

    timerOpen->stop();

    READCOUNT = 0;

    ui->SliderFrame->setEnabled(false);
    ui->SliderSec->setEnabled(false);
    cvReleaseImage(&imageRead);
    cvDestroyAllWindows();
}

void MainWindow::Read()
{
    READCOUNT++;

    ui->SliderFrame->setValue(READCOUNT);

    imageRead = cvQueryFrame(capture);
    cvShowImage("Play",imageRead);

    if(READCOUNT >= numFrames) // if avi ends
    {
        timerOpen->stop();
        ui->Stop->setEnabled(false);
        ui->Pause->setEnabled(false);
        ui->Play->setEnabled(true);
        READCOUNT = 0;
        cvReleaseImage(&imageRead);
        cvDestroyAllWindows();

    }
}

void MainWindow::SliderFramePress()
{

    timerOpen->stop();

}

void MainWindow::SliderFrameRelease()
{

        timerOpen->start((int) (1000/FPS_open));

}

void MainWindow::SliderFrameMove(int k)
{
    cvSetCaptureProperty(capture,CV_CAP_PROP_POS_FRAMES,k);

    imageRead = cvQueryFrame(capture);
    cvShowImage("Play",imageRead);

    READCOUNT = k;
}
