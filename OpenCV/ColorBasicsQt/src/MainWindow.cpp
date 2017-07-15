#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/videoio/videoio.hpp"

//#define BOOST_THREAD_PROVIDES_FUTURE
//#include <boost/thread.hpp>
//#include <boost/thread/future.hpp>
//#include <functional>
//#include <iostream>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>


using namespace std;

RGB* bufferDeCor;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        frequencia = double(qpf.QuadPart);
    }

    int milliseconds = 25;
    sensor = new KinectSensor(this, milliseconds);
    connect(sensor, SIGNAL(kinectCallback(RGBQUAD*, KinectTime*)),
            this, SLOT(onKinectCallback(RGBQUAD*, KinectTime*)));
}

cv::VideoWriter outputVideo;

void MainWindow::on_pushButton_clicked()
{
    cv::Size size = cv::Size(1920, 1080);
    int xvid = CV_FOURCC_MACRO('X','V','I','D');
    outputVideo.open("final.avi", xvid, 30, size, true);
    //int fourcc = CV_FOURCC('H','2','6','4');
    //int fmp4 = CV_FOURCC_MACRO('F','M','P','4');
    //outputVideo.open("saida.mp4", fmp4, 30, size);
    //int mp4v = CV_FOURCC_MACRO('M','P','4','V');
    //outputVideo.open("saida.mp4", mp4v, 30, size, false);
    //int mjpg = CV_FOURCC_MACRO('M','J','P','G');
    //outputVideo.open("saida.mp4", mjpg, 30, size);

    sensor->_deveCapturar = true;
    sensor->start();


//    _videoWriter = cv::VideoWriter("track.mp4", CV_FOURCC_MACRO('F','M','P','4'), 30.0, size, true);
//    _videoWriter = cv::VideoWriter("track.mp4", CV_FOURCC_MACRO('M','P','4','V'), 30.0, size, false);
//    _videoWriter = cv::VideoWriter("track.mp4", CV_FOURCC_MACRO('M','J','P','G'), 30.0, size);
//    _videoWriter = cv::VideoWriter("track.mp4", CV_FOURCC_MACRO('X','V','I','D'), 30.0, size);

//    int fourcc = VideoWriter.fourcc('M', 'J', 'P', 'G');
//    videoCapture.set(Videoio.CAP_PROP_FOURCC, fourcc);
//    videoCapture.set(Videoio.CAP_PROP_FRAME_WIDTH, CAP_FRAME_WIDTH);
//    videoCapture.set(Videoio.CAP_PROP_FRAME_HEIGHT, CAP_FRAME_HEIGHT);
}

void MainWindow::on_pushButton_2_clicked()
{
    sensor->_deveCapturar = false;
    outputVideo.release();
}

void KinectWriter(cv::VideoWriter writer, RGBQUAD *buffer, INT64 delay)
{
    cv::Mat saida(1080, 1920, CV_8UC4, buffer);
    cv::cvtColor(saida, saida, CV_BGRA2BGR);
    writer << saida;
    boost::this_thread::sleep_for(boost::chrono::milliseconds{delay});
}

void MainWindow::onKinectCallback(RGBQUAD *buffer, KinectTime *time)
{
    cv::Mat saida(1080, 1920, CV_8UC4, buffer);
    cv::cvtColor(saida, saida, CV_BGRA2BGR);
    outputVideo << saida;
//    if (sensor->_deveCapturar)
//    {
//        boost::thread{KinectWriter, outputVideo, buffer, time->valor};
//    }


    FPS(ui->lblFps, time->valor);
    ui->label->setPixmap(QPixmap::fromImage(QImage((uchar*) buffer,
      cColorWidth, cColorHeight, QImage::Format_RGB32)).scaledToWidth(720));


    //cv::Mat saida(buffer);
    //cv::cvtColor(buffer, buffer, CV_BGR2RGB);

    //cv::Mat frmCorRGB = cv::Mat(1080, 1920, CV_8UC4, buffer);
    //buffer.copyTo(frmCorRGB);

    //CvMat* matrix = buffer;
    // = cv::cvarrToMat(matrix);//Mat(1080, 1920, CV_8UC4, &buffer.clone());
    //cv::Mat frmSaida(1080, 1920, CV_8UC4, buffer);
    //cv::cvtColor(frmCorRGB, frmCorRGB, cv::COLOR_BGRA2BGR);
    //cv::Mat frmFinal = cv::Mat(1080, 1920, CV_8UC4, buffer->data);
//    cv::Mat frmFinal = cv::Mat(1080, 1920, CV_8UC4, buffer);
//    //frmFinal.data = buffer->data;
//    cv::Mat outro;
//    cv::cvtColor(frmFinal, outro, CV_BGRA2BGR);

    //cv::Mat frmImage = cv::Mat.zeros(1080, 1920, CV_8UC4);
    //cv::cvtColor(frmSaida, frmColor, CV_BGRA2BGR);
//        QImage result = QImage((const unsigned char*)(converted.data),
//                               converted.cols, converted.rows, QImage::Format_RGB888);
//        QLabel::setPixmap(QPixmap::fromImage(result));

}

void MainWindow::FPS(QLabel *label, INT64 time)
{
    if (!timeInicial)
    {
        timeInicial = time;
    }

    double fps = 0.0;
    LARGE_INTEGER qpcNow = {0};
    if (frequencia)
    {
        if (QueryPerformanceCounter(&qpcNow))
        {
            if (ultimaContagem)
            {
                totalDeframesAtualizados++;
                fps = frequencia * totalDeframesAtualizados /
                    double(qpcNow.QuadPart - ultimaContagem);
            }
        }
    }

    if (fps < 20)
    {
        label->setStyleSheet("color: red");
    }
    else
    {
        label->setStyleSheet("color: green");
    }

    INT64 now = GetTickCount64();
    if (proximaAtualizacaoTime <= now)
    {
        label->setText(QString::number(fps, 'f', 1) + "fps");
        proximaAtualizacaoTime = now + 1000; //1s
        ultimaContagem = qpcNow.QuadPart;
        totalDeframesAtualizados = 0;
    }
}

MainWindow::~MainWindow()
{
    delete ui;
    sensor->_deveCapturar = false;
    sensor = NULL;
    ///Sleeper::msleep(100);
    QThread::msleep(100);
    //QThread::currentThread()->wait();

}
