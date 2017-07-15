#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QTimer>
#include <QFileDialog>
#include <QMouseEvent>
#include <vector>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QGridLayout>

#pragma warning(push)
#pragma warning(disable:6255)
#pragma warning(disable:6263)
#pragma warning(disable:4995)
#include "ppl.h"
#pragma warning(pop)

using namespace std;
using namespace cv;

//cv::Point centro;

QImage mat_to_qimage_ref(cv::Mat &mat, QImage::Format format);

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->displayLatencia->display(QTime(0, 0).toString());
    ui->lcdDisplayPlay->display(QTime(0, 0).toString());
    ui->lblFPS->setStyleSheet("color: red");
    ui->tabViewEsquerda->tabBar()->hide();

    QVBoxLayout* layoutHSV = ui->layFiltrosHSV;
    QVBoxLayout* layoutRGB = ui->layFiltrosRGB;
    QFont fonte( "MS Shell Dlg 2", 10, QFont::Bold);

    QLabel* lblVermelho = new QLabel(tr("Vermelho"));
    lblVermelho->setFont(fonte);
    layoutRGB->addWidget(lblVermelho, 0, Qt::AlignTop);

    ctkRangeSlider* sliderVermelho = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderVermelho->connect(sliderVermelho, SIGNAL(valuesChanged(int, int)), this, SLOT(ajustarVermelho(int,int)));
    layoutRGB->addWidget(sliderVermelho, 0, Qt::AlignTop);
    sliderVermelho->setMinimum(0);
    sliderVermelho->setMaximum(255);
    sliderVermelho->setPositions(150, 255);

    QLabel* lblVerde = new QLabel(tr("Verde"));
    lblVerde->setFont(fonte);
    layoutRGB->addWidget(lblVerde, 0, Qt::AlignTop);

    ctkRangeSlider* sliderVerde = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderVerde->connect(sliderVerde, SIGNAL(valuesChanged(int, int)), this, SLOT(ajustarVerde(int,int)));
    layoutRGB->addWidget(sliderVerde, 0, Qt::AlignTop);
    sliderVerde->setMinimum(0);
    sliderVerde->setMaximum(255);
    sliderVerde->setPositions(150, 255);

    QLabel* lblAzul = new QLabel(tr("Azul"));
    lblAzul->setFont(fonte);
    layoutRGB->addWidget(lblAzul, 0, Qt::AlignTop);

    ctkRangeSlider* sliderAzul = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderAzul->connect(sliderAzul, SIGNAL(valuesChanged(int, int)), this, SLOT(ajustarAzul(int,int)));
    layoutRGB->addWidget(sliderAzul, 0, Qt::AlignTop);
    sliderAzul->setMinimum(0);
    sliderAzul->setMaximum(255);
    sliderAzul->setPositions(150, 255);

    QLabel* lblMatiz = new QLabel(tr("Matiz"));
    lblMatiz->setFont(fonte);
    layoutHSV->addWidget(lblMatiz, 0, Qt::AlignTop);

    ctkRangeSlider* sliderMatiz = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderMatiz->connect(sliderMatiz, SIGNAL(valuesChanged(int, int)), this, SLOT(ajustarMatiz(int,int)));
    layoutHSV->addWidget(sliderMatiz, 0, Qt::AlignTop);
    sliderMatiz->setMinimum(0);
    sliderMatiz->setMaximum(255);
    sliderMatiz->setPositions(0, 100);

    QLabel* lblSaturar = new QLabel(tr("Saturação"));
    lblSaturar->setFont(fonte);
    layoutHSV->addWidget(lblSaturar, 0, Qt::AlignTop);

    ctkRangeSlider* sliderSaturar = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderSaturar->connect(sliderSaturar, SIGNAL(valuesChanged(int, int)), this, SLOT(ajustarSaturacao(int,int)));
    layoutHSV->addWidget(sliderSaturar, 0, Qt::AlignTop);
    sliderSaturar->setMinimum(0);
    sliderSaturar->setMaximum(255);
    sliderSaturar->setPositions(0, 100);

    QLabel* lblValor = new QLabel(tr("Brilho"));
    lblValor->setFont(fonte);
    layoutHSV->addWidget(lblValor, 0, Qt::AlignTop);

    ctkRangeSlider* sliderValor = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderValor->connect(sliderValor, SIGNAL(valuesChanged(int, int)), this, SLOT(ajustarValor(int,int)));
    layoutHSV->addWidget(sliderValor, 0, Qt::AlignTop);
    sliderValor->setMinimum(0);
    sliderValor->setMaximum(255);
    sliderValor->setPositions(0, 250);

    QVBoxLayout* layoutContraste = ui->layFiltrosContraste;

    QLabel* lblContraste1 = new QLabel(tr("Contraste"));
    lblContraste1->setFont(fonte);
    layoutContraste->addWidget(lblContraste1, 0, Qt::AlignTop);

    ctkRangeSlider* sliderContraste1 = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderContraste1->connect(sliderContraste1, SIGNAL(valuesChanged(int, int)), this, SLOT(ajustarContraste(int,int)));
    layoutContraste->addWidget(sliderContraste1, 0, Qt::AlignTop);
    sliderContraste1->setMinimum(0);
    sliderContraste1->setMaximum(255);
    sliderContraste1->setPositions(150, 255);

    cv::setUseOptimized(true);

    inicializarSensorKinect();

    timeInicial = 0;
    ultimaContagem = 0;
    totalDeframesAtualizados = 0;
    frequencia = 0;

    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        frequencia = double(qpf.QuadPart);
    }

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(atualizarFrame()));
    timer->start(33);

    //cor
    _1920x1080_cor_buffer = new RGBQUAD[1920 * 1080];
    _1304x1080_cor_buffer = new RGBQUAD[1304 * 1080];
    _512x424_cor_buffer   = new RGBQUAD[512  *  424];
    //colorBuffer.resize(1920 * 1080 * 4);
    //mDepthImg(1080, 1920, CV_16UC1);
    frameColor = cv::Mat::zeros(1080, 1920, CV_8UC4);
    //infra
    _512x424_infraInt_buffer = new UINT16 [512 * 424];
    _512x424_infraCor_buffer = new RGBQUAD[512 * 424];

    //depth
    _512x424_depthInt_buffer = new UINT16 [512 * 424];
    _512x424_depthCor_buffer = new RGBQUAD[512 * 424];
    depthBuffer.resize(512 * 424);

    connect(&_timer, SIGNAL(timeout()), this, SLOT(atualizarLCD()));
    connect(&_timerPlay, SIGNAL(timeout()), this, SLOT(atualizarDisplayPlay()));
    connect(ui->btnVisualizar, SIGNAL(clicked(bool)), this, SLOT(abrirTrack()));
    connect(ui->btnCapturar, SIGNAL(clicked(bool)), this, SLOT(gravarTrack()));
    connect(ui->chkPararAuto, SIGNAL(clicked(bool)), this, SLOT(setPararGravacaoAuto()));
    connect(ui->tabViews, SIGNAL(currentChanged(int)), this, SLOT(alterarStatus()));
    connect(ui->sliderCirculoPlay, SIGNAL(valueChanged(int)), this, SLOT(dimensionarCirculo(int)));
    connect(ui->sliderCirculoX, SIGNAL(valueChanged(int)), this, SLOT(posicionarX(int)));
    connect(ui->sliderCirculoY, SIGNAL(valueChanged(int)), this, SLOT(posicionarY(int)));
    //connect(ui->sliderContrastePlay, SIGNAL(valueChanged(int)), this, SLOT(ajustarContraste(int)));
    connect(ui->chkInvertido, SIGNAL(clicked(bool)), this, SLOT(inverterContraste()));
    connect(ui->chkTrack, SIGNAL(clicked(bool)), this, SLOT(quantificar()));

    connect(ui->rdbOriginal, SIGNAL(toggled(bool)), this, SLOT(toggled()));
    connect(ui->rdbHSV, SIGNAL(toggled(bool)), this, SLOT(toggled()));
    connect(ui->rdbRGB, SIGNAL(toggled(bool)), this, SLOT(toggled()));
    connect(ui->rdbMascara, SIGNAL(toggled(bool)), this, SLOT(toggled()));


    //tocarTrack("D:/Qt/CapturaMorris/tracks/trk_20170412165411.knt");

    //namedWindow(hsvWindow);

    // Wait a Few Seconds until begins to Retrieve Data from Sensor ( about 2000-[ms] )
    std::this_thread::sleep_for(std::chrono::seconds(2));

}

void MainWindow::toggled()
{
    if (ui->rdbOriginal->isChecked())
    {
        ui->tabViewEsquerda->setCurrentIndex(0);
        ui->lblPrimeira->setText("HSV");
        ui->lblSegunda->setText("RGB");
        ui->lblTerceira->setText("Contraste");
    }
    else if (ui->rdbHSV->isChecked())
    {
        ui->tabViewEsquerda->setCurrentIndex(1);
        ui->lblPrimeira->setText("Original");
        ui->lblSegunda->setText("RGB");
        ui->lblTerceira->setText("Contraste");
    }
    else if (ui->rdbRGB->isChecked())
    {
        ui->tabViewEsquerda->setCurrentIndex(2);
        ui->lblPrimeira->setText("Original");
        ui->lblSegunda->setText("HSV");
        ui->lblTerceira->setText("Contraste");
    }
    else if (ui->rdbMascara->isChecked())
    {
        ui->tabViewEsquerda->setCurrentIndex(3);
        ui->lblPrimeira->setText("Original");
        ui->lblSegunda->setText("HSV");
        ui->lblTerceira->setText("RGB");
    }
}


void MainWindow::ajustarVerde(int min, int max)
{
    _minVerde = min;
    _maxVerde = max;

}

void MainWindow::ajustarVermelho(int min, int max)
{
    _minVermelho = min;
    _maxVermelho = max;
}

void MainWindow::ajustarAzul(int min, int max)
{
    _minAzul = min;
    _maxAzul = max;
}

void MainWindow::ajustarMatiz(int min, int max)
{
    _minMatiz = min;
    _maxMatiz = max;

}

void MainWindow::ajustarSaturacao(int min, int max)
{
    _minSaturar = min;
    _maxSaturar = max;
}

void MainWindow::ajustarValor(int min, int max)
{
    _minBrilho = min;
    _maxBrilho = max;
}

void MainWindow::ajustarContraste(int min, int max)
{
    _minContraste = min;
    _maxContraste = max;
}


cv::Mat lambda(2, 4, CV_32FC1);
std::vector<std::vector<cv::Point>> tetragons;
//int THRESHOLD_ANIMAL_VS_FLOOR = 240;
int THRESHOLD_WALL_VS_FLOOR = 80;
cv::RNG rng(12345);
cv::Mat frmRastros = cv::Mat::zeros(424, 512, CV_8UC3);
cv::Mat frameLine  = cv::Mat::zeros(424, 512, CV_8UC3);
cv::Mat frmRastrosFinal  = cv::Mat::zeros(424, 512, CV_8UC3);
//cv::Mat frameColor = cv::Mat::zeros(424, 512, CV_8UC4);
cv::Mat frmCinza, frmHSV, frmResult, frmMascara, frameTemp, frmEspelhado, frameRedondo, frmDesfoco, frmContraste;


cv::Scalar preto  = cv::Scalar(0, 0, 0);
cv::Scalar branco = cv::Scalar(255,255,255);
cv::Scalar vermelho = cv::Scalar(0,0,255);
cv::Scalar amarelo = cv::Scalar(30,255,255);
cv::Scalar cinza = cv::Scalar(150,150,150);
cv::Scalar cinzaEscuro = cv::Scalar(75,75,75);
cv::Scalar azul = cv::Scalar(255, 0, 0);
cv::Scalar verde = cv::Scalar(0, 255, 0);



void MainWindow::posicionarX(int valor)
{
    ui->sliderCirculoX->setMinimum(_raio);
    ui->sliderCirculoX->setMaximum(512 - _raio);

    if(valor >= _raio && valor <= (512 - _raio))
    {
       _eixoX = valor;
    }
    //centro = cv::Point(&_eixoX,&_eixoY);
}

void MainWindow::posicionarY(int valor)
{
    ui->sliderCirculoY->setMinimum(_raio);
    ui->sliderCirculoY->setMaximum(424 - _raio);

    if(valor >= _raio && valor <= (424 - _raio))// (_raio * 2))
    {
       _eixoY = valor;
    }

    //centro = cv::Point(_eixoX,valor);
}

void MainWindow::ajustarContraste(int valor)
{
    contraste = valor;
}


void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
       // _raio = 300;
        //&& iconLabel->geometry().contains(event->pos())) {

        //QDrag *drag = new QDrag(this);
        //QMimeData *mimeData = new QMimeData;

        //mimeData->setText(commentEdit->toPlainText());
        //drag->setMimeData(mimeData);
        //drag->setPixmap(iconPixmap);

        //Qt::DropAction dropAction = drag->exec();
        //...
    }
}

void MainWindow::dimensionarCirculo(int valor)
{
    _raio = valor;
    //ui->sliderCirculoPlay
//    ui->sliderCirculoX->setMinimum(valor/2);
//    ui->sliderCirculoY->setMinimum(valor/2);
//    ui->sliderCirculoX->setMaximum(valor*2);
//    ui->sliderCirculoY->setMaximum(valor*2);
}

// Create a HSV image from the RGB image using the full 8-bits, since OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
// Remember to free the generated HSV image.
IplImage* convertImageRGBtoHSV(const IplImage *imageRGB)
{
    float fR, fG, fB;
    float fH, fS, fV;
    const float FLOAT_TO_BYTE = 255.0f;
    const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;

    // Create a blank HSV image
    IplImage *imageHSV = cvCreateImage(cvGetSize(imageRGB), 8, 3);
    if (!imageHSV || imageRGB->depth != 8 || imageRGB->nChannels != 3) {
        printf("ERROR in convertImageRGBtoHSV()! Bad input image.\n");
        exit(1);
    }

    int h = imageRGB->height;		// Pixel height.
    int w = imageRGB->width;		// Pixel width.
    int rowSizeRGB = imageRGB->widthStep;	// Size of row in bytes, including extra padding.
    char *imRGB = imageRGB->imageData;	// Pointer to the start of the image pixels.
    int rowSizeHSV = imageHSV->widthStep;	// Size of row in bytes, including extra padding.
    char *imHSV = imageHSV->imageData;	// Pointer to the start of the image pixels.
    for (int y=0; y<h; y++) {
        for (int x=0; x<w; x++) {
            // Get the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
            uchar *pRGB = (uchar*)(imRGB + y*rowSizeRGB + x*3);
            int bB = *(uchar*)(pRGB+0);	// Blue component
            int bG = *(uchar*)(pRGB+1);	// Green component
            int bR = *(uchar*)(pRGB+2);	// Red component

            // Convert from 8-bit integers to floats.
            fR = bR * BYTE_TO_FLOAT;
            fG = bG * BYTE_TO_FLOAT;
            fB = bB * BYTE_TO_FLOAT;

            // Convert from RGB to HSV, using float ranges 0.0 to 1.0.
            float fDelta;
            float fMin, fMax;
            int iMax;
            // Get the min and max, but use integer comparisons for slight speedup.
            if (bB < bG) {
                if (bB < bR) {
                    fMin = fB;
                    if (bR > bG) {
                        iMax = bR;
                        fMax = fR;
                    }
                    else {
                        iMax = bG;
                        fMax = fG;
                    }
                }
                else {
                    fMin = fR;
                    fMax = fG;
                    iMax = bG;
                }
            }
            else {
                if (bG < bR) {
                    fMin = fG;
                    if (bB > bR) {
                        fMax = fB;
                        iMax = bB;
                    }
                    else {
                        fMax = fR;
                        iMax = bR;
                    }
                }
                else {
                    fMin = fR;
                    fMax = fB;
                    iMax = bB;
                }
            }
            fDelta = fMax - fMin;
            fV = fMax;				// Value (Brightness).
            if (iMax != 0) {			// Make sure it's not pure black.
                fS = fDelta / fMax;		// Saturation.
                float ANGLE_TO_UNIT = 1.0f / (6.0f * fDelta);	// Make the Hues between 0.0 to 1.0 instead of 6.0
                if (iMax == bR) {		// between yellow and magenta.
                    fH = (fG - fB) * ANGLE_TO_UNIT;
                }
                else if (iMax == bG) {		// between cyan and yellow.
                    fH = (2.0f/6.0f) + ( fB - fR ) * ANGLE_TO_UNIT;
                }
                else {				// between magenta and cyan.
                    fH = (4.0f/6.0f) + ( fR - fG ) * ANGLE_TO_UNIT;
                }
                // Wrap outlier Hues around the circle.
                if (fH < 0.0f)
                    fH += 1.0f;
                if (fH >= 1.0f)
                    fH -= 1.0f;
            }
            else {
                // color is pure Black.
                fS = 0;
                fH = 0;	// undefined hue
            }

            // Convert from floats to 8-bit integers.
            int bH = (int)(0.5f + fH * 255.0f);
            int bS = (int)(0.5f + fS * 255.0f);
            int bV = (int)(0.5f + fV * 255.0f);

            // Clip the values to make sure it fits within the 8bits.
            if (bH > 255)
                bH = 255;
            if (bH < 0)
                bH = 0;
            if (bS > 255)
                bS = 255;
            if (bS < 0)
                bS = 0;
            if (bV > 255)
                bV = 255;
            if (bV < 0)
                bV = 0;

            // Set the HSV pixel components.
            uchar *pHSV = (uchar*)(imHSV + y*rowSizeHSV + x*3);
            *(pHSV+0) = bH;		// H component
            *(pHSV+1) = bS;		// S component
            *(pHSV+2) = bV;		// V component
        }
    }
    return imageHSV;
}


// Create an RGB image from the HSV image using the full 8-bits, since OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
// Remember to free the generated RGB image.
IplImage* convertImageHSVtoRGB(const IplImage *imageHSV)
{
    float fH, fS, fV;
    float fR, fG, fB;
    const float FLOAT_TO_BYTE = 255.0f;
    const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;

    // Create a blank RGB image
    IplImage *imageRGB = cvCreateImage(cvGetSize(imageHSV), 8, 3);
    if (!imageRGB || imageHSV->depth != 8 || imageHSV->nChannels != 3) {
        printf("ERROR in convertImageHSVtoRGB()! Bad input image.\n");
        exit(1);
    }

    int h = imageHSV->height;			// Pixel height.
    int w = imageHSV->width;			// Pixel width.
    int rowSizeHSV = imageHSV->widthStep;		// Size of row in bytes, including extra padding.
    char *imHSV = imageHSV->imageData;		// Pointer to the start of the image pixels.
    int rowSizeRGB = imageRGB->widthStep;		// Size of row in bytes, including extra padding.
    char *imRGB = imageRGB->imageData;		// Pointer to the start of the image pixels.
    for (int y=0; y<h; y++) {
        for (int x=0; x<w; x++) {
            // Get the HSV pixel components
            uchar *pHSV = (uchar*)(imHSV + y*rowSizeHSV + x*3);
            int bH = *(uchar*)(pHSV+0);	// H component
            int bS = *(uchar*)(pHSV+1);	// S component
            int bV = *(uchar*)(pHSV+2);	// V component

            // Convert from 8-bit integers to floats
            fH = (float)bH * BYTE_TO_FLOAT;
            fS = (float)bS * BYTE_TO_FLOAT;
            fV = (float)bV * BYTE_TO_FLOAT;

            // Convert from HSV to RGB, using float ranges 0.0 to 1.0
            int iI;
            float fI, fF, p, q, t;

            if( bS == 0 ) {
                // achromatic (grey)
                fR = fG = fB = fV;
            }
            else {
                // If Hue == 1.0, then wrap it around the circle to 0.0
                if (fH >= 1.0f)
                    fH = 0.0f;

                fH *= 6.0;			// sector 0 to 5
                fI = floor( fH );		// integer part of h (0,1,2,3,4,5 or 6)
                iI = (int) fH;			//		"		"		"		"
                fF = fH - fI;			// factorial part of h (0 to 1)

                p = fV * ( 1.0f - fS );
                q = fV * ( 1.0f - fS * fF );
                t = fV * ( 1.0f - fS * ( 1.0f - fF ) );

                switch( iI ) {
                    case 0:
                        fR = fV;
                        fG = t;
                        fB = p;
                        break;
                    case 1:
                        fR = q;
                        fG = fV;
                        fB = p;
                        break;
                    case 2:
                        fR = p;
                        fG = fV;
                        fB = t;
                        break;
                    case 3:
                        fR = p;
                        fG = q;
                        fB = fV;
                        break;
                    case 4:
                        fR = t;
                        fG = p;
                        fB = fV;
                        break;
                    default:		// case 5 (or 6):
                        fR = fV;
                        fG = p;
                        fB = q;
                        break;
                }
            }

            // Convert from floats to 8-bit integers
            int bR = (int)(fR * FLOAT_TO_BYTE);
            int bG = (int)(fG * FLOAT_TO_BYTE);
            int bB = (int)(fB * FLOAT_TO_BYTE);

            // Clip the values to make sure it fits within the 8bits.
            if (bR > 255)
                bR = 255;
            if (bR < 0)
                bR = 0;
            if (bG > 255)
                bG = 255;
            if (bG < 0)
                bG = 0;
            if (bB > 255)
                bB = 255;
            if (bB < 0)
                bB = 0;

            // Set the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
            uchar *pRGB = (uchar*)(imRGB + y*rowSizeRGB + x*3);
            *(pRGB+0) = bB;		// B component
            *(pRGB+1) = bG;		// G component
            *(pRGB+2) = bR;		// R component
        }
    }
    return imageRGB;
}


void MainWindow::showImage(Mat img, QLabel *lbl)
{
    if(img.empty()) {
        cout<<"Empty image to show!!!"<<endl;
        return;
    }
    QImage Qimgbkg;
    if(img.type()==CV_8UC3) {
        cvtColor(img,img,CV_BGR2RGB);
        QImage Qimgbkg1((uchar*)img.data,img.cols,img.rows,img.step,QImage::Format_RGB888);
        Qimgbkg = Qimgbkg1;
    }
    else if(img.type()==CV_8UC4) {
        cvtColor(img,img,CV_BGRA2RGBA);
        QImage Qimgbkg1((uchar*)img.data,img.cols,img.rows,img.step,QImage::Format_RGBA8888);
        Qimgbkg = Qimgbkg1;
    }
    else {
        QImage Qimgbkg1((uchar*)img.data,img.cols,img.rows,img.step,QImage::Format_Indexed8);
        Qimgbkg=Qimgbkg1;
    }

    lbl->setPixmap(QPixmap::fromImage(Qimgbkg));
}


void printType(cv::Mat frm) {
  string str;

  int type = frm.type();
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  str = "8U"; break;
    case CV_8S:  str = "8S"; break;
    case CV_16U: str = "16U"; break;
    case CV_16S: str = "16S"; break;
    case CV_32S: str = "32S"; break;
    case CV_32F: str = "32F"; break;
    case CV_64F: str = "64F"; break;
    default:     str = "User"; break;
  }

  str += "C";
  str += (chans+'0');

  printf("Matrix: %s %dw x %dh \n", str.c_str(), frm.cols, frm.rows);
}

void MainWindow::tocarTrack(QString nomeDoArquivo)
{
    if (!nomeDoArquivo.isEmpty())
    {
        nomeArquivoAtual = nomeDoArquivo;

        int totalDeBytes = 512 * 424 * sizeof(RGBQUAD);

        QThread *thread;
        thread = new QThread;
        KinectReader *kinectReader;
        std::string nome = nomeArquivoAtual.toUtf8().constData();
        kinectReader = new KinectReader(nome, totalDeBytes,  _512x424_cor_buffer);
        kinectReader->moveToThread(thread);
        connect(thread, SIGNAL(started()), kinectReader, SLOT(processar()));
        connect(kinectReader, SIGNAL(processado()), this, SLOT(atualizarTela()));
        connect(kinectReader, SIGNAL(concluido()), thread, SLOT(quit()));
        thread->start();

        ui->lcdDisplayPlay->display(QTime(0, 0).toString());
        _cron.reiniciar();
        _cronPlay.reiniciar();
        _timerPlay.start(33);
    }
}


void MainWindow::adicionarRegistro(cv::Mat frm)
{
    // Adicionar o registro
    cv::circle(frm, cv::Point(_eixoX, _eixoY), _raio, branco, 2);
    cv::circle(frm, cv::Point(_eixoX, _eixoY), _raio - 35, branco, 1);
    //cv::circle(frm, cv::Point(_eixoX, _eixoY), _raio - 15, verde, 1);

    if (ui->rdbNO->isChecked())
    {
        //NO
        cv::circle(frm, cv::Point((_eixoX - (_raio / 3)), (_eixoY - (_raio / 3))),  _raio / 9, branco, 2);
        //cv::circle(frm, cv::Point(((eixoX - (raio / 3)) + 18), ((eixoY - (raio / 3))- 20)), raio / 9, corInterna, 1);
    }
    else if (ui->rdbNE->isChecked())
    {
        //NE
        cv::circle(frm, cv::Point((_eixoX + (_raio / 3)), (_eixoY - (_raio / 3))),  _raio / 9, branco, 2);
    }
    else if (ui->rdbSE->isChecked())
    {
        //SE
        cv::circle(frm, cv::Point((_eixoX + (_raio / 3)), (_eixoY + (_raio / 3))),  _raio / 9, branco, 2);
    }
    else if (ui->rdbSO->isChecked())
    {
        //SO
        cv::circle(frm, cv::Point((_eixoX - (_raio / 3)), (_eixoY + (_raio / 3))),  _raio / 9, branco, 2);
    }


    cv::line(frm, cv::Point(_eixoX, (_eixoY - _raio - 20)), cv::Point(_eixoX, (_eixoY + _raio + 20)), branco, 1, cv::LINE_8);
    cv::line(frm, cv::Point((_eixoX - _raio - 20), _eixoY), cv::Point((_eixoX + _raio + 20), _eixoY), branco, 1, cv::LINE_8);
}

double distanceCalculate(double x1, double y1, double x2, double y2)
{
    double x = x1 - x2; //calculating number to square in next step
    double y = y1 - y2;
    double dist;

    dist = pow(x, 2) + pow(y, 2); //calculating Euclidean distance
    dist = sqrt(dist);

    return dist;
}

void MainWindow::atualizarTela()
{
    // Iniciar cronômetro
    INT64 _playTime = _cronPlay.getTime().msec();
    FPS(ui->lblFPSPlay, _playTime);
    _cronPlay.reiniciar();

    // Criar frame colorido
    cv::Mat frmCor(424, 512, CV_8UC4, _512x424_cor_buffer);
    cv::Mat frmCorRGB, frmMascara, frmMascaraFinal, frmCinza,
            frmCinzaInRange, frmContraste, frmContrasteFinal,
            frmBGR2HSV, frmHSVInRange, frmHSVContraste, frmHSVFinal;

    cv::cvtColor(frmCor, frmCorRGB, cv::COLOR_BGR2RGB);
    cv::cvtColor(frmCor, frmBGR2HSV, cv::COLOR_BGR2HSV);
    cv::cvtColor(frmCor, frmCinza, cv::COLOR_BGR2GRAY);

    //cv::threshold(frmCinza, frmContraste, 240, 255, cv::THRESH_BINARY);

    //C++: void inpaint(InputArray src, InputArray inpaintMask, OutputArray dst, double inpaintRadius, int flags)
    /**
        Parameters:
        src – Input 8-bit 1-channel or 3-channel image.
        inpaintMask – Inpainting mask, 8-bit 1-channel image. Non-zero pixels indicate the area that needs to be inpainted.
        dst – Output image with the same size and type as src .
        inpaintRadius – Radius of a circular neighborhood of each point inpainted that is considered by the algorithm.
        flags –
        Inpainting method that could be one of the following:

        INPAINT_NS Navier-Stokes based method.
        INPAINT_TELEA Method by Alexandru Telea [Telea04].
    */
    //cv::inpaint(frmCorRGB, frmContraste, frmMascaraFinal, 3, INPAINT_NS); //INPAINT_NS

    //frmCorRGB[100,100] = [255,255,255];

    //cv::inRange(frmCorRGB, cv::Scalar(200, 200, 200), cv::Scalar(255, 255, 255), frmMascara);
    //frmCorRGB.setTo(frmMascaraFinal, frmMascara);

//    int oldValue = 255;
//    int newValue = 0;

    //frmCorRGB.setTo(newValue, frmCorRGB == oldValue);

    // Aplicar o RGB
    cv::Scalar minRGB = cv::Scalar(_minAzul, _minVerde, _minVermelho);
    cv::Scalar maxRGB = cv::Scalar(_maxAzul, _maxVerde, _maxVermelho);
    cv::inRange(frmCorRGB, minRGB, maxRGB, frmCinzaInRange);

    // Define range in HSV and apply threshold
    //http://www.shervinemami.info/colorConversion.html
    cv::Scalar minHSV = cv::Scalar(_minMatiz, _minSaturar, _minBrilho);
    cv::Scalar maxHSV = cv::Scalar(_maxMatiz, _maxSaturar, _maxBrilho);
    cv::inRange(frmBGR2HSV, minHSV, maxHSV, frmHSVInRange);

    // Criar máscara cinza
    cv::Mat frmCinzaElipse(424, 512, CV_8UC1, preto);
    cv::ellipse(frmCinzaElipse, cv::Point(_eixoX, _eixoY), cv::Size(_raio, _raio), 0, 0, 360, branco, -1, 8);
    cv::Mat frmCinzaRedondo(424, 512, CV_8UC1);
    cv::bitwise_and(frmCinzaInRange, frmCinzaElipse, frmCinzaRedondo);

    cv::Mat frmCinzaRedondo2(424, 512, CV_8UC1);
    cv::bitwise_and(frmCinza, frmCinzaElipse, frmCinzaRedondo2);

    cv::Mat frmCinzaRedondo3(424, 512, CV_8UC1);  //frmHSVInRange
    cv::bitwise_and(frmHSVInRange, frmCinzaElipse, frmCinzaRedondo3);

    //C++: void fastNlMeansDenoising(InputArray src, OutputArray dst, float h=3,
    //                               int templateWindowSize=7, int searchWindowSize=21 )
    //          fastNlMeansDenoisingColored(), fastNlMeansDenoisingMulti(), fastNlMeansDenoisingColoredMulti()
    /**



    Parameters:
    src – Input 8-bit 1-channel, 2-channel or 3-channel image.
    dst – Output image with the same size and type as src .
    templateWindowSize – Size in pixels of the template patch that is used to compute weights.
          Should be odd. Recommended value 7 pixels
    searchWindowSize – Size in pixels of the window that is used to compute weighted average for
          given pixel. Should be odd. Affect performance linearly: greater searchWindowsSize -
          greater denoising time. Recommended value 21 pixels
    h – Parameter regulating filter strength. Big h value perfectly removes noise but also
          removes image details, smaller h value preserves details but also preserves some noise
    */

    //cv::fastNlMeansDenoising(frmCinzaRedondo, frmCinzaRedondo);

    //Simple Thresholding
    //cv::THRESH_BINARY
    //cv::THRESH_BINARY_INV
    //cv::THRESH_TRUNC
    //cv::THRESH_TOZERO
    //cv::THRESH_TOZERO_INV
    //cv::threshold(frmCinzaRedondo, frmMascara, _minContraste, _maxContraste, cv::THRESH_TRUNC);

    cv::inRange(frmCinzaRedondo, Scalar(_minContraste), Scalar(_maxContraste), frmMascara);
    cv::Mat frmPreto1(frmCinzaRedondo.size(), CV_8U, Scalar(0));
    frmPreto1.copyTo(frmCinzaRedondo, frmMascara);

    cv::inRange(frmCinzaRedondo2, Scalar(_minContraste), Scalar(_maxContraste), frmContraste);
    cv::Mat frmPreto2(frmCinzaRedondo2.size(), CV_8U, Scalar(0));
    frmPreto2.copyTo(frmCinzaRedondo2, frmContraste);

    cv::inRange(frmCinzaRedondo3, Scalar(_minContraste), Scalar(_maxContraste), frmHSVContraste);
    cv::Mat frmPreto3(frmCinzaRedondo3.size(), CV_8U, Scalar(0));
    frmPreto3.copyTo(frmCinzaRedondo3, frmHSVContraste);

    // cv::MORPH_ELLIPSE, cv::MORPH_CROSS, cv::MORPH_RECT
    int type = cv::MORPH_ELLIPSE;

    // Create a structuring element
//    int erosion_size = 2;
//    cv::Mat elementErode = cv::getStructuringElement(type,
//          cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
//          cv::Point(erosion_size, erosion_size) );
//    cv::erode(frmHSVContraste, frmHSVContraste, elementErode);

    // Criar frame com desfoque
    //cv::Mat frmDesfoco = cv::Mat::zeros(424, 512, CV_8UC1);
    //cv::GaussianBlur(frmHSVContraste, frmHSVContraste, cv::Size(5,5), 0);

    //cv::bilateralFilter(frmHSVContraste, frmHSVContraste,9,75,75);

    //Mat img;
    //int imgs_count = 5;
    //fastNlMeansDenoisingColoredMulti(buffer, img, imgs_count / 2, imgs_count, 12, 48);
    //cv::fastNlMeansDenoisingMulti(buffer, img, imgs_count / 2, imgs_count, 12, 48);

    int morph_elem = 0;
    int morph_size = 2;
    //Opening: MORPH_OPEN : 2
    //Closing: MORPH_CLOSE: 3
    //Gradient: MORPH_GRADIENT: 4
    //Top Hat: MORPH_TOPHAT: 5
    //Black Hat: MORPH_BLACKHAT: 6
    Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ),
                                         Point( morph_size, morph_size ) );
    cv::morphologyEx(frmHSVContraste, frmHSVContraste, cv::MORPH_CLOSE, element);

//    int dilation_size = 5;
//    cv::Mat elementDilate = cv::getStructuringElement(type,
//         cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
//         cv::Point( dilation_size, dilation_size ));
//    cv::dilate(frmHSVContraste, frmHSVContraste, elementDilate);

    cv::cvtColor(frmHSVContraste, frmHSVFinal, CV_GRAY2BGR);

    int blur = 25;
    cv::GaussianBlur(frmHSVContraste, frmHSVContraste, cv::Size(blur, blur), 0);

    //Simple Thresholding
    //cv::THRESH_BINARY
    //cv::THRESH_BINARY_INV
    //cv::THRESH_TRUNC
    //cv::THRESH_TOZERO
    //cv::THRESH_TOZERO_INV
    //cv::threshold(frmCinzaRedondo, frmMascara, _minContraste, _maxContraste, cv::THRESH_TRUNC);
    int THRESHOLD_ANIMAL_VS_FLOOR = 120;
    cv::threshold(frmHSVContraste, frmHSVContraste, THRESHOLD_ANIMAL_VS_FLOOR, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frmHSVContraste, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    std::vector<int> indices(contours.size());
    std::iota(indices.begin(), indices.end(), 0);

    //O primeiro contorno no índice será o maior.
    std::sort(indices.begin(), indices.end(), [&contours](int lhs, int rhs) {
        return contours[lhs].size() > contours[rhs].size();
    });

    size_t totalDeIndices = indices.size();
    if (totalDeIndices > 0){

       //frameTemp = cv::Mat::zeros(424, 512, CV_8UC4);
       cv::drawContours(frmCor, contours, indices[0], azul, 2);
       cv::drawContours(frmHSVFinal, contours, indices[0], azul, 2);

       //cv::drawContours(frmHSVContraste, contours, indices[0], cv::Scalar(150, 150, 150), 2);
       //cv::drawContours(frmMascaraFinal, contours, indices[0], cv::Scalar(150, 150, 150), 2);

        std::vector<cv::Point> contour = contours[0];
        cv::Moments mmts = moments(contour,false);
        int mx = (int)(mmts.m10 / mmts.m00);
        int my = (int)(mmts.m01 / mmts.m00);

        if (mx > 0 && my > 0)
        {
            // Desenha o centroid.
            cv::Point centroid = cv::Point(mx, my);
            cv::circle(frmHSVFinal, centroid, 2, azul, -1);

            if (_x > 0 && _y > 0)
            {
                //QTime cronCalc = _cronCalc.getTime();
                //QTime _1s = QTime(0,0,0,950);
                //if (cronCalc.operator >=(_1s))
                //{
                    //2,14 px por cm;
                    double diametro = double(_raio * 2);
                    double px_por_cm = (diametro / double(168));
                    double x = (mx - _x);
                    double y = (my - _y);
                    double xSqr = double(x * x);
                    double ySqr = double(y * y);
                    double distanciaMedida = sqrt(xSqr + ySqr);
                    //distancia += distanceCalculate(mx, my, double(_x), double(_x));
                    distancia_em_cm += round(distanciaMedida / px_por_cm);
                    //distancia_em_cm += floor(distanciaTemp * 100) / 100;
                    //double distanciaProxi = roundf(distancia_em_cm * 100) / 100;
                    //double distanciaAcima = ceilf(distancia_em_cm * 100) / 100;

                    //cv::Point2f a(mx, my);
                    //cv::Point2f b(_x, _y);
                    //distancia += cv::norm(a-b);
                    _cronCalc.reiniciar();
                //}

                cv::Point origem = cv::Point(_x, _y);
                cv::line(frameLine, centroid, origem, verde, 1.5);
                cv::add(frameLine, frmRastros, frmRastros);

                string strDistancia = "D " + boost::lexical_cast<std::string>(distancia_em_cm) +"cm";
                cv::putText(frmCorRGB, strDistancia, cv::Point(390,30), cv::FONT_HERSHEY_DUPLEX, 0.7, vermelho);

                cv::addWeighted(frmCorRGB, 0.7, frmRastros, 1, 1, frmCorRGB);
                cv::addWeighted(frmHSVFinal, 0.7, frmRastros, 1, 1, frmHSVFinal);
            }
            _x = centroid.x;
            _y = centroid.y;
        }
    }

    if (ui->rdbOriginal->isChecked())
    {
        adicionarRegistro(frmCorRGB);
        printType(frmCorRGB);
        QImage image = QImage(frmCorRGB.data, 512, 424, QImage::Format_RGB888);
        ui->lblViewResultadoPlay->setPixmap(QPixmap::fromImage(image));
    }

    if (ui->rdbHSV->isChecked())
    {
        //cv::cvtColor(frmHSVContraste, frmHSVFinal, CV_GRAY2BGR);
        adicionarRegistro(frmHSVFinal);
        cv::cvtColor(frmHSVFinal,frmHSVFinal,CV_BGR2RGB);
        QImage image = QImage(frmHSVFinal.data, 512, 424, QImage::Format_RGB888);
        ui->lblViewResultadoPlay->setPixmap(QPixmap::fromImage(image));
    }

    if (ui->rdbRGB->isChecked())
    {
        cv::cvtColor(frmMascara, frmMascaraFinal, CV_GRAY2BGR);
        adicionarRegistro(frmMascaraFinal);
        cv::cvtColor(frmMascaraFinal,frmMascaraFinal, CV_BGR2RGB);
        QImage image = QImage(frmMascaraFinal.data, 512, 424, QImage::Format_RGB888);
        ui->lblViewResultadoPlay->setPixmap(QPixmap::fromImage(image));
    }

    if (ui->rdbMascara->isChecked())
    {
        cv::cvtColor(frmContraste, frmContrasteFinal, CV_GRAY2BGR);
        adicionarRegistro(frmContrasteFinal);
        cv::cvtColor(frmContrasteFinal, frmContrasteFinal, CV_BGR2RGB);
        QImage image = QImage(frmContrasteFinal.data, 512, 424, QImage::Format_RGB888);
        ui->lblViewResultadoPlay->setPixmap(QPixmap::fromImage(image));
    }

    // thumbs
    if (ui->rdbOriginal->isChecked())
    {
        cv::cvtColor(frmHSVContraste, frmHSVContraste, CV_GRAY2RGB);
        QImage img1 = QImage(frmHSVContraste.data, 512, 424, QImage::Format_RGB888);
        ui->vPrimeira->setPixmap(QPixmap::fromImage(img1).scaledToWidth(205));

        cv::cvtColor(frmMascara, frmMascaraFinal, CV_GRAY2RGB);
        QImage img2 = QImage(frmMascaraFinal.data, 512, 424, QImage::Format_RGB888);
        ui->vSegunda->setPixmap(QPixmap::fromImage(img2).scaledToWidth(205));

        cv::cvtColor(frmContraste, frmContrasteFinal, CV_GRAY2RGB);
        QImage img3 = QImage(frmContrasteFinal.data, 512, 424, QImage::Format_RGB888);
        ui->vTerceira->setPixmap(QPixmap::fromImage(img3).scaledToWidth(205));
    }
    else if (ui->rdbHSV->isChecked())
    {
        QImage img1 = QImage(frmCor.data, 512, 424, QImage::Format_RGB32);
        ui->vPrimeira->setPixmap(QPixmap::fromImage(img1).scaledToWidth(205));

        cv::cvtColor(frmMascara, frmMascaraFinal, CV_GRAY2RGB);
        QImage img2 = QImage(frmMascaraFinal.data, 512, 424, QImage::Format_RGB888);
        ui->vSegunda->setPixmap(QPixmap::fromImage(img2).scaledToWidth(205));

        cv::cvtColor(frmContraste, frmContrasteFinal, CV_GRAY2RGB);
        QImage img3 = QImage(frmContrasteFinal.data, 512, 424, QImage::Format_RGB888);
        ui->vTerceira->setPixmap(QPixmap::fromImage(img3).scaledToWidth(205));
    }
    else if (ui->rdbRGB->isChecked())
    {
        QImage img1 = QImage(frmCor.data, 512, 424, QImage::Format_RGB32);
        ui->vPrimeira->setPixmap(QPixmap::fromImage(img1).scaledToWidth(205));

        cv::cvtColor(frmHSVContraste, frmHSVContraste, CV_GRAY2RGB);
        QImage img2 = QImage(frmHSVContraste.data, 512, 424, QImage::Format_RGB888);
        ui->vSegunda->setPixmap(QPixmap::fromImage(img2).scaledToWidth(205));

        cv::cvtColor(frmContraste, frmContrasteFinal, CV_GRAY2RGB);
        QImage img3 = QImage(frmContrasteFinal.data, 512, 424, QImage::Format_RGB888);
        ui->vTerceira->setPixmap(QPixmap::fromImage(img3).scaledToWidth(205));
    }
    else if (ui->rdbMascara->isChecked())
    {
        QImage img1 = QImage(frmCor.data, 512, 424, QImage::Format_RGB32);
        ui->vPrimeira->setPixmap(QPixmap::fromImage(img1).scaledToWidth(205));

        cv::cvtColor(frmHSVContraste, frmHSVContraste, CV_GRAY2RGB);
        QImage img2 = QImage(frmHSVContraste.data, 512, 424, QImage::Format_RGB888);
        ui->vSegunda->setPixmap(QPixmap::fromImage(img2).scaledToWidth(205));

        cv::cvtColor(frmMascara, frmMascaraFinal, CV_GRAY2RGB);
        QImage img3 = QImage(frmMascaraFinal.data, 512, 424, QImage::Format_RGB888);
        ui->vTerceira->setPixmap(QPixmap::fromImage(img3).scaledToWidth(205));
    }
}

void MainWindow::quantificar()
{
    if (deveQuantificar == true)
    {
        ui->chkTrack->setChecked(false);
        deveQuantificar = false;
    }
    else
    {
        ui->chkTrack->setChecked(true);
        deveQuantificar = true;
    }
}



void MainWindow::inverterContraste()
{
    if (_invertido == true)
    {
        ui->chkInvertido->setChecked(false);
        _invertido = false;
    }
    else
    {
        ui->chkInvertido->setChecked(true);
        _invertido = true;
    }
}

void MainWindow::alterarStatus()
{
    if(ui->tabViews->currentIndex() == 0)
    {
       _deveProcessar = true;
    }
    else if(ui->tabViews->currentIndex() == 1)
    {
       _deveProcessar = false;
    }
}

void MainWindow::setPararGravacaoAuto()
{
    if (_pararGravacaoAuto == true)
    {
        ui->chkPararAuto->setChecked(false);
        _pararGravacaoAuto = false;
    }
    else
    {
        ui->chkPararAuto->setChecked(true);
        _pararGravacaoAuto = true;
    }

}

void MainWindow::atualizarLCD()
{
    ui->displayLatencia->display(_cron.getTime().toString());
    ui->lcdDisplayPlay->display(_cron.getTime().toString());
    QTime cronTime = _cron.getTime();
    QTime maxTime = ui->cboTimerLimite->time();
    QTime maxTimePlay = ui->cboTimerPlay->time();
    if (_pararGravacaoAuto == true && cronTime.operator >=(maxTime) || cronTime.operator >=(maxTimePlay))
    {
        definirTela();
    }
}

void MainWindow::gravarTrack()
{
    _deveProcessar = true;

    BOOLEAN isSensorConectado = false;
    HRESULT hr = sensorKinect->get_IsAvailable(&isSensorConectado);
    if (isSensorConectado == TRUE)
    {
        if (_deveGravar == true)
        {
            definirTela();
        }
        else
        {
            ui->btnCapturar->setText("Parar");
            ui->btnCapturar->setStyleSheet("background-color: red");
            ui->displayLatencia->display(QTime(0, 0).toString());
            _deveGravar = true;
            _cron.reiniciar();
            _timer.start(1000);
        }
    }
}

void MainWindow::definirTela()
{
    ui->btnCapturar->setText("Gravar");
    ui->btnCapturar->setStyleSheet("background-color: green");
    _deveGravar = false;
    _timer.stop();
}


void MainWindow::atualizarDisplayPlay()
{
    ui->lcdDisplayPlay->display(_cron.getTime().toString());
    //QTime cronTime = _cron.getTime();
    QTime cronStop = _cronPlay.getTime();
    QTime _200ms = QTime(0,0,0,200);
    //QTime maxTimePlay = ui->cboTimerPlay->time();
    if (cronStop.operator >=(_200ms)){
        _timerPlay.stop();
    }
}

void MainWindow::abrirTrack()
{
    QString filtros = " Kinect Stream (*.knt);;"
                      " Todos (*.*)";

    QString path = QDir::currentPath() + "/../tracks";
    QString nomeDoArquivo = QFileDialog::getOpenFileName(
                this,
                tr("Abrir arquivo"),
                path,
                filtros);

    if (!nomeDoArquivo.isEmpty())
    {
        frmRastros = cv::Mat::zeros(424, 512, CV_8UC3);
        frameLine  = cv::Mat::zeros(424, 512, CV_8UC3);
        _x = 0, _y = 0, distancia_em_cm = 0;
        nomeArquivoAtual = nomeDoArquivo;
        tocarTrack(nomeArquivoAtual);
    }
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

    std::string str = (boost::format("FPS %1$.0f ") % fps).str();
    label->setText(QString(str.c_str()));

    INT64 now = GetTickCount64();
    if (proximaAtualizacaoTime <= now)
    {
        proximaAtualizacaoTime = now + 1000; //1s
        ultimaContagem = qpcNow.QuadPart;
        totalDeframesAtualizados = 0;
    }
}

HRESULT MainWindow::inicializarSensorKinect()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&sensorKinect);
    if (FAILED(hr))
    {
        return hr;
    }

    if (sensorKinect)
    {
        hr = sensorKinect->get_CoordinateMapper(&mapper);
        hr = sensorKinect->Open();
        hr = sensorKinect->OpenMultiSourceFrameReader(
                FrameSourceTypes::FrameSourceTypes_Depth |
                FrameSourceTypes::FrameSourceTypes_Color |
                FrameSourceTypes::FrameSourceTypes_Infrared,
                &msFrameReader);
    }

    if (!sensorKinect || FAILED(hr))
    {
        return E_FAIL;
    }

    return hr;
}

void MainWindow::atualizarFrame()
{
    if (!msFrameReader || !_deveProcessar)
    {
        return;
    }

    IMultiSourceFrame* multiSrcFrm = NULL;
    HRESULT hr = msFrameReader->AcquireLatestFrame(&multiSrcFrm);

    if (SUCCEEDED(hr))
    {
        // Processar cor
        bool deveProcessarCor = ui->rdb2Dcinza->isChecked();
        if (deveProcessarCor)
        {
            IColorFrameReference* colorFrmRef = NULL;
            hr = multiSrcFrm->get_ColorFrameReference(&colorFrmRef);

            if (SUCCEEDED(hr))
            {
                IColorFrame* colorFrm = NULL;
                hr = colorFrmRef->AcquireFrame(&colorFrm);

                if (SUCCEEDED(hr))
                {
                    UINT colorBufferSize = 1920 * 1080 * sizeof(RGBQUAD);
                    UINT uBufferSize = 1080 * 1920 * 4 * sizeof(BYTE);

                    ColorImageFormat imageFormat = ColorImageFormat_None;
                    hr = colorFrm->get_RelativeTime(&_colorTime);
                    hr = colorFrm->get_RawColorImageFormat(&imageFormat);

                    //colorFrm->CopyConvertedFrameDataToArray(uBufferSize, frameColor.data, ColorImageFormat_Bgra);
                    //colorFrm->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()),
                    //        &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra);

                    if (imageFormat == ColorImageFormat_Bgra)
                    {
                        hr = colorFrm->AccessRawUnderlyingBuffer(&colorBufferSize, reinterpret_cast<BYTE**>(&_1920x1080_cor_buffer));
                    }
                    else
                    {
                        hr = colorFrm->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(_1920x1080_cor_buffer), ColorImageFormat_Bgra);
                    }
                    processarCor();
                }
                SafeRelease(colorFrm);
            }
            SafeRelease(colorFrmRef);
        }

        // Processar frame infravermelho
        bool deveProcessarInfraVermelho = ui->rdb2Dinfra->isChecked();
        if (deveProcessarInfraVermelho){

            IInfraredFrameReference* infraFrmRef = NULL;
            hr = multiSrcFrm->get_InfraredFrameReference(&infraFrmRef);

            if (SUCCEEDED(hr))
            {
                IInfraredFrame* infraFrm = NULL;
                hr = infraFrmRef->AcquireFrame(&infraFrm);

                if (SUCCEEDED(hr))
                {
                    UINT infraBufferSize = 0;
                    hr = infraFrm->get_RelativeTime(&_infraTime);
                    hr = infraFrm->AccessUnderlyingBuffer(&infraBufferSize, &_512x424_infraInt_buffer);
                    processarInfrared();
                }
                SafeRelease(infraFrm);
            }
            SafeRelease(infraFrmRef);
        }

        // Processar frame tridimensional
        bool deveProcessar3d = ui->rdb3Dcinza->isChecked();
        if (deveProcessar3d)
        {
            IDepthFrameReference* depthFrmRef = NULL;
            hr = multiSrcFrm->get_DepthFrameReference(&depthFrmRef);

            if (SUCCEEDED(hr))
            {
                IDepthFrame* depthFrm = NULL;
                hr = depthFrmRef->AcquireFrame(&depthFrm);

                if (SUCCEEDED(hr))
                {
                    UINT depthBufferSize = 0;
                    hr = depthFrm->get_RelativeTime(&_depthTime);
                    hr = depthFrm->AccessUnderlyingBuffer(&depthBufferSize, &_512x424_depthInt_buffer);
                    processarDepth();
                }
                SafeRelease(depthFrm);
            }
            SafeRelease(depthFrmRef);
        }

        if (_deveGravar)
        {
            QString nomeDoArquivo = ui->txtIdentificacao->toPlainText();
            std::string str = nomeDoArquivo.toUtf8().constData();

            UINT size = (512 * 424 * sizeof(RGBQUAD));

            bool gravarCor = ui->rdb2Dcinza->isChecked();
            if (gravarCor)
            {
                kinectWriter.gravarFrame(str, _512x424_cor_buffer, size);

                //UINT cvSize = 424 * 424 * 4 * sizeof(BYTE);
                //kinectWriter.gravarFrameCv(str, frameColor, cvSize);
                //kinectWriter.SaveMatBinary(str, frameColor);
            }

            bool gravarInfraVermelho = ui->rdb2Dinfra->isChecked();
            if (gravarInfraVermelho)
            {
                kinectWriter.gravarFrame(str, _512x424_infraCor_buffer, size);
            }

            bool gravar3d = ui->rdb3Dcinza->isChecked();
            if (gravar3d){
                kinectWriter.gravarFrame(str, _512x424_depthCor_buffer, size);
            }
        }
        else if (kinectWriter.isAbertoParaGravar()){
            kinectWriter.fecharArquivoSeAberto();
        }
    }
    else
    {
       ui->lblFPS->setStyleSheet("color: red");
       ui->lblFPS->setText("Offline");
    }
    SafeRelease(multiSrcFrm);
}

QImage mat_to_qimage_ref(cv::Mat &mat, QImage::Format format)
{
  return QImage(mat.data, mat.cols, mat.rows, format);
}

void MainWindow::processarCor()
{
    FPS(ui->lblFPS, _colorTime);

    //cropping
    int margin = (1920 - 1304) / 2;
    Concurrency::parallel_for(0, 1080, [&](int y)
    {
        int outIndex = 1304 * y;
        for (int x = margin; x < (1304 + margin); ++x, ++outIndex)
        {
            int inIndex = (y * 1920) + x;
            _1304x1080_cor_buffer[outIndex] = _1920x1080_cor_buffer[inIndex];
        }
    });

    //downsampling
    float fator = (float) 1080 / (float) 424;
    Concurrency::parallel_for(0, 424, [&](int y)
    {
        unsigned int index = 512 * y;
        for (int x = 0; x < 512; ++x, ++index)
        {
            int srcX = (int)(x * fator);
            int srcY = (int)(y * fator);
            int srcIndex = (srcY * 1304) + srcX;
            _512x424_cor_buffer[index] = _1304x1080_cor_buffer[srcIndex];
        }
    });

    //mirroring
    Concurrency::parallel_for(0, 424, [&](int y)
    {
        int index = y * 512;
        int mirrorIndex = index + 512 - 1;

        for (int x = 0; x < (512 / 2); ++x, ++index, --mirrorIndex)
        {
            RGBQUAD pixel = _512x424_cor_buffer[index];
            _512x424_cor_buffer[index] = _512x424_cor_buffer[mirrorIndex];
            _512x424_cor_buffer[mirrorIndex] = pixel;
        }
    });


    //INICIO DO RASTREAMENTO--------------------------------

    if (deveQuantificar == true)
    {
        int size = colorBuffer.size();
        cv::Mat colorImage(424, 512, CV_8UC4, _512x424_cor_buffer);
        //cv::Mat showImage;
        //cv::resize(colorImage, showImage, cv::Size(512 / 2, 1080 / 2));

        QImage image = mat_to_qimage_ref(colorImage, QImage::Format_ARGB32);
        //ui->lblViewOriginal->setPixmap(QPixmap::fromImage(image).scaledToWidth(205));

        //cores
        cv::Scalar preto  = cv::Scalar(0, 0, 0);
        cv::Scalar branco = cv::Scalar(255,255,255);

        cv::Mat frmCinza, frmEspelhado, frameRedondo, frmDesfoco, frmConstraste;

        //frame cor
        //cv::Mat frameCropped = colorImage(cv::Rect(420, 0, 1080, 1080));
        //cv::resize(frameCropped, frmRedimensionado, cv::Size(1080 / 2.55f, 1080 / 2.55f));
        cv::flip(colorImage, frmEspelhado, 1);
        //frmResultado = frmEspelhado;

        //QImage imgOriginal = mat_to_qimage_ref(frmRedimensionado, QImage::Format_ARGB32);
        //ui->lblViewOriginal->setPixmap(QPixmap::fromImage(imgOriginal).scaledToWidth(190));

        int threshold = cv::THRESH_BINARY;
        if (_invertido == true)
        {
            threshold = cv::THRESH_BINARY_INV;
        }

        //frame redondo
        cv::Point centro = cv::Point(256,212);
        cv::Mat frameMask(frmEspelhado.rows, frmEspelhado.cols, CV_8UC4, preto);
        cv::ellipse(frameMask, centro, cv::Size(_raio,_raio), 0, 0, 360, branco, -1, 8);
        cv::bitwise_and(frmEspelhado, frameMask, frameRedondo);

        //frame contraste
        cv::cvtColor(frameRedondo, frmCinza, cv::COLOR_BGR2GRAY);

        QImage imgCinza = mat_to_qimage_ref(frmCinza, QImage::Format_Grayscale8);
        //ui->lblViewCinza->setPixmap(QPixmap::fromImage(imgCinza).scaledToWidth(205));

        cv::GaussianBlur(frmCinza, frmDesfoco, cv::Size(25,25), 0);

        //(const Mat& src, Mat& dst, double thresh, double maxVal, int thresholdType)
        //src – Source array (single-channel, 8-bit of 32-bit floating point)
        //dst – Destination array; will have the same size and the same type as src
        //thresh – Threshold value
        //maxVal – Maximum value to use with THRESH_BINARY and THRESH_BINARY_INV thresholding types
        //thresholdType – Thresholding type (see the discussion)
        cv::threshold(frmDesfoco, frmConstraste, contraste, 255, threshold);

        QImage imgContraste = mat_to_qimage_ref(frmConstraste, QImage::Format_Grayscale8);
        //ui->lblViewContraste->setPixmap(QPixmap::fromImage(imgContraste).scaledToWidth(205));

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(frmConstraste, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        std::vector<int> indices(contours.size());
        std::iota(indices.begin(), indices.end(), 0);

        //O primeiro contorno no índice será o maior.
        std::sort(indices.begin(), indices.end(), [&contours](int lhs, int rhs) {
            return contours[lhs].size() > contours[rhs].size();
        });

        cv::Mat frmTmp;
        if (indices.size() > 0){

           frmTmp = cv::Mat::zeros(424, 512, CV_8UC4);
           cv::drawContours(frmRastros, contours, indices[0], cv::Scalar(255, 0, 0), 10);
           //cv::drawContours(frmResultado, contours, indices[1], cv::Scalar(255, 0, 0), 5);

           //cv::circle(frmResultado, cv::Point(180,180),140, cv::Scalar(0,0,255), 10);
           cv::circle(frmTmp, cv::Point(256,212), 200, cv::Scalar(0,0,255), 10);

            std::vector<cv::Point> contour = contours[0];
            cv::Moments mmts = moments(contour,false);
            int mx = (int)(mmts.m10 / mmts.m00);
            int my = (int)(mmts.m01 / mmts.m00);

            if (mx > 0 && my > 0)
            {
                cv::Point centroid = cv::Point(mx, my);
                if (_x > 0 && _y > 0)
                {
                    cv::Point origem = cv::Point(_x, _y);
                    cv::line(frameLine, centroid, origem, cv::Scalar(255, 255, 255), 10);
                    cv::add(frameLine, frmRastros, frmRastros);
                    cv::addWeighted(frmTmp, 0.7, frmRastros, 1, 1, frmTmp);
                }
                _x = centroid.x;
                _y = centroid.y;
            }
        }

        QImage imgRastros = mat_to_qimage_ref(frmRastros, QImage::Format_Grayscale8);
        //ui->lblViewRastreamento->setPixmap(QPixmap::fromImage(imgRastros).scaledToWidth(205));
    }
    //FIM DO RASTREAMENTO--------------------------------

    ui->lblCapturar->setPixmap(QPixmap::fromImage(QImage((uchar*)
        _512x424_cor_buffer, 512, 424, QImage::Format_RGB32)).scaledToWidth(640));
}

/**
void MainWindow::processarCor()
{
    FPS(ui->lblFPS, _colorTime);

    //cores
    cv::Scalar preto  = cv::Scalar(0, 0, 0);
    cv::Scalar branco = cv::Scalar(255,255,255);

    cv::Mat frmResultado, frmCinza, frmRedimensionado, frmEspelhado,
            frameRedondo, frmDesfoco, frmConstraste;

    //frame cor
    cv::Mat frameCropped = frameColor(cv::Rect(420, 0, 1080, 1080));
    cv::resize(frameCropped, frmRedimensionado, cv::Size(1080 / 2.55f, 1080 / 2.55f));
    cv::flip(frmRedimensionado, frmEspelhado, 1);
    frmResultado = frmEspelhado;

    QImage imgOriginal = mat_to_qimage_ref(frmRedimensionado, QImage::Format_ARGB32);
    ui->lblViewOriginal->setPixmap(QPixmap::fromImage(imgOriginal).scaledToWidth(190));

    int threshold = cv::THRESH_BINARY;
    if (_invertido == true)
    {
        threshold = cv::THRESH_BINARY_INV;
    }

    //frame redondo
    cv::Point centro = cv::Point(212,212);
    cv::Mat frameMask(frmEspelhado.rows, frmEspelhado.cols, CV_8UC4, preto);
    cv::ellipse(frameMask, centro, raio, 0, 0, 360, branco, -1, 8);
    cv::bitwise_and(frmEspelhado,frameMask,frameRedondo);

    //frame contraste
    cv::cvtColor(frameRedondo, frmCinza, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(frmCinza, frmDesfoco, cv::Size(25,25), 0);

    //(const Mat& src, Mat& dst, double thresh, double maxVal, int thresholdType)
    //src – Source array (single-channel, 8-bit of 32-bit floating point)
    //dst – Destination array; will have the same size and the same type as src
    //thresh – Threshold value
    //maxVal – Maximum value to use with THRESH_BINARY and THRESH_BINARY_INV thresholding types
    //thresholdType – Thresholding type (see the discussion)
    cv::threshold(frmDesfoco, frmConstraste, contraste, 255, threshold);

    QImage imgContraste = mat_to_qimage_ref(frmConstraste, QImage::Format_Grayscale8);
    ui->lblViewContraste->setPixmap(QPixmap::fromImage(imgContraste).scaledToWidth(190));

    //INICIO DO RASTREAMENTO--------------------------------

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frmConstraste, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    std::vector<int> indices(contours.size());
    std::iota(indices.begin(), indices.end(), 0);

    //O primeiro contorno no índice será o maior.
    std::sort(indices.begin(), indices.end(), [&contours](int lhs, int rhs) {
        return contours[lhs].size() > contours[rhs].size();
    });

    if (indices.size() > 0){

       cv::Mat frmTmp = cv::Mat::zeros(424, 424, CV_8UC1);
       //cv::drawContours(frmRastros, contours, indices[0], cv::Scalar(255, 0, 0), 5);
       cv::drawContours(frmResultado, contours, indices[1], cv::Scalar(255, 0, 0), 5);

       cv::circle(frmResultado, cv::Point(180,180),140, cv::Scalar(0,0,255), 10);
       cv::circle(frmTmp, cv::Point(180,180),140, cv::Scalar(0,0,255), 10);

        std::vector<cv::Point> contour = contours[0];
        cv::Moments mmts = moments(contour,false);
        int mx = (int)(mmts.m10 / mmts.m00);
        int my = (int)(mmts.m01 / mmts.m00);

        if (mx > 0 && my > 0)
        {
            cv::Point centroid = cv::Point(mx, my);
            if (_x > 0 && _y > 0)
            {
                cv::Point origem = cv::Point(_x, _y);
                cv::line(frameLine, centroid, origem, cv::Scalar(255, 255, 255), 10);
                cv::add(frameLine, frmRastros, frmRastros);
                cv::addWeighted(frmTmp, 0.7, frmRastros, 1, 1, frmTmp);
            }
            _x = centroid.x;
            _y = centroid.y;
        }
    }
    //FIM DO RASTREAMENTO--------------------------------

    QImage imgRastros = mat_to_qimage_ref(frmRastros, QImage::Format_Grayscale8);
    ui->lblViewRastreamento->setPixmap(QPixmap::fromImage(imgRastros).scaledToWidth(190));

    QImage imgCinza = mat_to_qimage_ref(frmCinza, QImage::Format_Grayscale8);
    ui->lblViewCinza->setPixmap(QPixmap::fromImage(imgCinza).scaledToWidth(190));

    QImage imgResultado = mat_to_qimage_ref(frmResultado, QImage::Format_ARGB32);
    ui->lblViewResultado->setPixmap(QPixmap::fromImage(imgResultado));
}
*/

void MainWindow::processarInfrared()
{
    FPS(ui->lblFPS, _infraTime);

    RGBQUAD* tempBuffer = _512x424_infraCor_buffer;
    const UINT16* totalBuffer = _512x424_infraInt_buffer + (512 * 424);

    //mapping
    while (_512x424_infraInt_buffer < totalBuffer)
    {
        float intensityRatio = static_cast<float>(*_512x424_infraInt_buffer) / InfraredSourceValueMaximum;
        intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviations;
        intensityRatio = min(InfraredOutputValueMaximum, intensityRatio);
        intensityRatio = max(InfraredOutputValueMinimum, intensityRatio);
        byte intensity = static_cast<byte>(intensityRatio * 255.0f);
        tempBuffer->rgbRed   = intensity;
        tempBuffer->rgbGreen = intensity;
        tempBuffer->rgbBlue  = intensity;
        ++tempBuffer;
        ++_512x424_infraInt_buffer;
    }

    //mirroring
    Concurrency::parallel_for(0, 424, [&](int y)
    {
        int index = y * 512;
        int mirrorIndex = index + 512 - 1;

        for (int x = 0; x < (512 / 2); ++x, ++index, --mirrorIndex)
        {
            RGBQUAD pixel = _512x424_infraCor_buffer[index];
            _512x424_infraCor_buffer[index] = _512x424_infraCor_buffer[mirrorIndex];
            _512x424_infraCor_buffer[mirrorIndex] = pixel;
        }
    });

    ui->lblCapturar->setPixmap(QPixmap::fromImage(QImage((uchar*)
        _512x424_infraCor_buffer, 512, 424, QImage::Format_RGB32)));
}

void MainWindow::processarDepth()
{

    FPS(ui->lblFPS, _depthTime);

    RGBQUAD* tempBuffer = _512x424_depthCor_buffer;
    const UINT16* totalBuffer = _512x424_depthInt_buffer + (512 * 424);

    while (_512x424_depthInt_buffer < totalBuffer)
    {
        USHORT depth = *_512x424_depthInt_buffer;

        // To convert to a byte, we're discarding the most-significant
        // rather than least-significant bits.
        // We're preserving detail, although the intensity will "wrap."
        // Values outside the reliable depth range are mapped to 0 (black).

        // Note: Using conditionals in this loop could degrade performance.
        // Consider using a lookup table instead when writing production code.
        BYTE intensity = static_cast<BYTE>((depth >= _minDepth)
                   && (depth <= _maxDepth) ? (depth % 256) : 0);

        tempBuffer->rgbRed   = intensity;
        tempBuffer->rgbGreen = intensity;
        tempBuffer->rgbBlue  = intensity;

        ++tempBuffer;
        ++_512x424_depthInt_buffer;
    }

    //mirroring
    Concurrency::parallel_for(0, 424, [&](int y)
    {
        int index = y * 512;
        int mirrorIndex = index + 512 - 1;

        for (int x = 0; x < (512 / 2); ++x, ++index, --mirrorIndex)
        {
            RGBQUAD pixel = _512x424_depthCor_buffer[index];
            _512x424_depthCor_buffer[index] = _512x424_depthCor_buffer[mirrorIndex];
            _512x424_depthCor_buffer[mirrorIndex] = pixel;
        }
    });

    ui->lblCapturar->setPixmap(QPixmap::fromImage(QImage((uchar*)
        _512x424_depthCor_buffer, 512, 424, QImage::Format_RGB32)));
}

MainWindow::~MainWindow()
{
    delete ui;

    if (_1920x1080_cor_buffer)
    {
        delete [] _1920x1080_cor_buffer;
        _1920x1080_cor_buffer = NULL;
    }

    if (_1304x1080_cor_buffer)
    {
        delete [] _1304x1080_cor_buffer;
        _1304x1080_cor_buffer = NULL;
    }

    if (_512x424_cor_buffer)
    {
        delete [] _512x424_cor_buffer;
        _512x424_cor_buffer = NULL;
    }

    if (_512x424_infraInt_buffer)
    {
        delete [] _512x424_infraInt_buffer;
        _512x424_infraInt_buffer = NULL;
    }

    if (_512x424_infraCor_buffer)
    {
        delete [] _512x424_infraCor_buffer;
        _512x424_infraCor_buffer = NULL;
    }


    if (_512x424_depthInt_buffer)
    {
        delete [] _512x424_depthInt_buffer;
        _512x424_depthInt_buffer = NULL;
    }

    if (_512x424_depthCor_buffer)
    {
        delete [] _512x424_depthCor_buffer;
        _512x424_depthCor_buffer = NULL;
    }

    SafeRelease(msFrameReader);
    SafeRelease(mapper);

    if (sensorKinect)
    {
        sensorKinect->Close();
    }

    SafeRelease(sensorKinect);
}
