#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QTimer>
#include <time.h>

string now() {
  char s[1024];
  time_t t = time(NULL);
  struct tm lnow;
  localtime_s(&lnow, &t);
  sprintf_s(s, "%04d-%02d-%02d_%02d-%02d-%02d", lnow.tm_year + 1900, lnow.tm_mon + 1, lnow.tm_mday, lnow.tm_hour, lnow.tm_min, lnow.tm_sec);
  return string(s);
}

NtKinect kinect;
int scale = 1;
cv::Size sz(1920/scale,1080/scale);
bool onSave = true;
cv::Mat img;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateKinectData()));
    timer->start(33);
}



void MainWindow::updateKinectData()
{
    kinect.setRGB();

    if (_deveGravar)
    {
      cv::resize(kinect.rgbImage, img, sz, 0, 0);
      cv::cvtColor(img, img, CV_BGRA2BGR);
      vw << img;
      if (!vw.isOpened()) throw runtime_error("cannot create video file");
    }

    ///Map to depth
//    kinect.setDepth();
//    for (int y=0; y<kinect.depthImage.rows; y++) {
//      for (int x=0; x<kinect.depthImage.cols; x++) {
//        UINT16 d = kinect.depthImage.at<UINT16>(y,x);
//        DepthSpacePoint dp; dp.X=x; dp.Y=y;
//        ColorSpacePoint cp;
//        HRESULT hr = kinect.coordinateMapper->MapDepthPointToColorSpace(dp,d,&cp);
//        if (hr != S_OK) continue;
//        if (d > 2000 || d < 500) {
//          int ax = (int) cp.X;
//          int ay = (int) cp.Y;
//          cv::rectangle(kinect.rgbImage,cv::Rect(ax-2,ay-2,4,4),cv::Scalar(255,0,0),2);
//        }
//      }
//    }

    cv::Mat frm = kinect.rgbImage;
    QImage qimage((const unsigned char*) frm.data, 1920, 1080, QImage::Format_RGB32);
    ui->label->setPixmap(QPixmap::fromImage(qimage.scaled(ui->label->frameSize())));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_btnGravar_clicked()
{
    _deveGravar = true;
    //vw = cv::VideoWriter(now()+".avi",CV_FOURCC_MACRO('X','V','I','D'), 30.0, sz);
    vw = cv::VideoWriter(now()+".mp4",CV_FOURCC_MACRO('F','M','P','4'), 30.0, sz);
    //vw = cv::VideoWriter(now()+".mp4",CV_FOURCC_MACRO('M','P','4','V'), 30.0, sz);

}

void MainWindow::on_btnParar_clicked()
{
    vw.release();
    _deveGravar = false;
}
