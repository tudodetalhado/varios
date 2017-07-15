#define _CRT_SECURE_NO_WARNINGS

#include "MainWindow.h"
#include "Common.h"
#include "ui_MainWindow.h"
#include "Constantes.h"
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

//boost::circular_buffer<cv::Mat> _queue(circularBufferSize);
//CircularBuffer<cv::Mat> _queue(circularBufferSize);

QSemaphore *freeSlots;
QSemaphore *usedSlots;

using namespace std;
using namespace cv;

int scale2 = 1;
cv::Size sz2(1920/scale2,1080/scale2);

//Cores OpenCV
cv::Scalar preto  = cv::Scalar(0, 0, 0);
cv::Scalar branco = cv::Scalar(255,255,255);
cv::Scalar vermelho = cv::Scalar(0,0,255);
cv::Scalar amarelo = cv::Scalar(30,255,255);
cv::Scalar cinza = cv::Scalar(150,150,150);
cv::Scalar cinzaEscuro = cv::Scalar(75,75,75);
cv::Scalar azul = cv::Scalar(255, 0, 0);
cv::Scalar verde = cv::Scalar(0, 255, 0);

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->displayLatencia->display(QTime(0, 0).toString());
    ui->lblFPS->setStyleSheet("color: black");

//    freeSlots = new QSemaphore(circularBufferSize);
//    usedSlots = new QSemaphore(0);

    QVBoxLayout* layoutHSV = ui->layFiltrosHSV;
    QVBoxLayout* layoutRGB = ui->layFiltrosRGB;
    QFont fonte( "MS Shell Dlg 2", 10, QFont::Bold);

    QLabel* lblVermelho = new QLabel(tr("Vermelho"));
    lblVermelho->setFont(fonte);
    layoutRGB->addWidget(lblVermelho, 0, Qt::AlignTop);

    ctkRangeSlider* sliderVermelho = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderVermelho->connect(sliderVermelho, SIGNAL(valuesChanged(int, int)),
                            this, SLOT(ajustarVermelho(int,int)));
    layoutRGB->addWidget(sliderVermelho, 0, Qt::AlignTop);
    sliderVermelho->setMinimum(0);
    sliderVermelho->setMaximum(255);
    sliderVermelho->setPositions(150, 255);

    QLabel* lblVerde = new QLabel(tr("Verde"));
    lblVerde->setFont(fonte);
    layoutRGB->addWidget(lblVerde, 0, Qt::AlignTop);

    ctkRangeSlider* sliderVerde = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderVerde->connect(sliderVerde, SIGNAL(valuesChanged(int, int)),
                         this, SLOT(ajustarVerde(int,int)));
    layoutRGB->addWidget(sliderVerde, 0, Qt::AlignTop);
    sliderVerde->setMinimum(0);
    sliderVerde->setMaximum(255);
    sliderVerde->setPositions(150, 255);

    QLabel* lblAzul = new QLabel(tr("Azul"));
    lblAzul->setFont(fonte);
    layoutRGB->addWidget(lblAzul, 0, Qt::AlignTop);

    ctkRangeSlider* sliderAzul = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderAzul->connect(sliderAzul, SIGNAL(valuesChanged(int, int)),
                        this, SLOT(ajustarAzul(int,int)));
    layoutRGB->addWidget(sliderAzul, 0, Qt::AlignTop);
    sliderAzul->setMinimum(0);
    sliderAzul->setMaximum(255);
    sliderAzul->setPositions(150, 255);

    QLabel* lblMatiz = new QLabel(tr("Matiz"));
    lblMatiz->setFont(fonte);
    layoutHSV->addWidget(lblMatiz, 0, Qt::AlignTop);

    ctkRangeSlider* sliderMatiz = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderMatiz->connect(sliderMatiz, SIGNAL(valuesChanged(int, int)),
                         this, SLOT(ajustarMatiz(int,int)));
    layoutHSV->addWidget(sliderMatiz, 0, Qt::AlignTop);
    sliderMatiz->setMinimum(0);
    sliderMatiz->setMaximum(255);
    sliderMatiz->setPositions(0, 100);

    QLabel* lblSaturar = new QLabel(tr("Saturação"));
    lblSaturar->setFont(fonte);
    layoutHSV->addWidget(lblSaturar, 0, Qt::AlignTop);

    ctkRangeSlider* sliderSaturar = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderSaturar->connect(sliderSaturar, SIGNAL(valuesChanged(int, int)),
                           this, SLOT(ajustarSaturacao(int,int)));
    layoutHSV->addWidget(sliderSaturar, 0, Qt::AlignTop);
    sliderSaturar->setMinimum(0);
    sliderSaturar->setMaximum(255);
    sliderSaturar->setPositions(0, 100);

    QLabel* lblValor = new QLabel(tr("Brilho"));
    lblValor->setFont(fonte);
    layoutHSV->addWidget(lblValor, 0, Qt::AlignTop);

    ctkRangeSlider* sliderValor = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderValor->connect(sliderValor, SIGNAL(valuesChanged(int, int)),
                         this, SLOT(ajustarValor(int,int)));
    layoutHSV->addWidget(sliderValor, 0, Qt::AlignTop);
    sliderValor->setMinimum(0);
    sliderValor->setMaximum(255);
    sliderValor->setPositions(0, 250);

    QVBoxLayout* layoutContraste = ui->layFiltrosContraste;

    QLabel* lblContraste1 = new QLabel(tr("Contraste"));
    lblContraste1->setFont(fonte);
    layoutContraste->addWidget(lblContraste1, 0, Qt::AlignTop);

    ctkRangeSlider* sliderContraste1 = new ctkRangeSlider(Qt::Horizontal, 0);
    sliderContraste1->connect(sliderContraste1, SIGNAL(valuesChanged(int, int)),
                              this, SLOT(ajustarContraste(int,int)));
    layoutContraste->addWidget(sliderContraste1, 0, Qt::AlignTop);
    sliderContraste1->setMinimum(0);
    sliderContraste1->setMaximum(255);
    sliderContraste1->setPositions(150, 255);

    cv::setUseOptimized(true);

    timeInicial = 0;
    ultimaContagem = 0;
    totalDeframesAtualizados = 0;
    frequencia = 0;

    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        frequencia = double(qpf.QuadPart);
    }

    //QTimer *timer = new QTimer(this);
    //connect(timer, SIGNAL(timeout()), this, SLOT(lerDadosSensorKinect()));
    //timer->start(33);

    //cor
    _1920x1080_cor_buffer = new RGBQUAD[1920 * 1080];
    _1304x1080_cor_buffer = new RGBQUAD[1304 * 1080];
    _512x424_cor_buffer   = new RGBQUAD[512  *  424];
    _640x530_cor_buffer   = new RGBQUAD[640  *  530];
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
    //connect(ui->btnVisualizar, SIGNAL(clicked(bool)), this, SLOT(abrirTrack()));
    connect(ui->btnCapturar, SIGNAL(clicked(bool)), this, SLOT(capturar()));
    connect(ui->chkPararAuto, SIGNAL(clicked(bool)), this, SLOT(setPararGravacaoAuto()));
    connect(ui->tabViewPrincipal, SIGNAL(currentChanged(int)), this, SLOT(alterarStatus()));
    connect(ui->sliderCirculoPlay, SIGNAL(valueChanged(int)), this, SLOT(dimensionarCirculo(int)));
    connect(ui->sliderCirculoX, SIGNAL(valueChanged(int)), this, SLOT(posicionarX(int)));
    connect(ui->sliderCirculoY, SIGNAL(valueChanged(int)), this, SLOT(posicionarY(int)));
    //connect(ui->sliderContrastePlay, SIGNAL(valueChanged(int)), this, SLOT(ajustarContraste(int)));
    connect(ui->chkInvertido, SIGNAL(clicked(bool)), this, SLOT(inverterContraste()));
    connect(ui->chkGravar, SIGNAL(clicked(bool)), this, SLOT(deveGravarVideoSlot()));

    //connect(ui->rdbOriginal, SIGNAL(toggled(bool)), this, SLOT(toggled()));
    //connect(ui->rdbHSV, SIGNAL(toggled(bool)), this, SLOT(toggled()));
    //connect(ui->rdbRGB, SIGNAL(toggled(bool)), this, SLOT(toggled()));
    //connect(ui->rdbMascara, SIGNAL(toggled(bool)), this, SLOT(toggled()));
    connect(ui->viewArena, SIGNAL(cliqueDuplo()), this, SLOT(abrirTrack()));
    connect(ui->viewCaptura, SIGNAL(cliqueDuplo()), this, SLOT(abrirTrack()));
    connect(ui->vPrimeira, SIGNAL(cliqueDuplo()), this, SLOT(abrirTrack()));
    connect(ui->vSegunda, SIGNAL(cliqueDuplo()), this, SLOT(abrirTrack()));
    connect(ui->vTerceira, SIGNAL(cliqueDuplo()), this, SLOT(abrirTrack()));
    connect(ui->btnSalvarArena, SIGNAL(clicked(bool)), this, SLOT(salvarArena()));
    connect(ui->viewArena, SIGNAL(desativarFerramentas()), this, SLOT(desmarcarFerramentas()));


    //Circular Buffer
    int deviceNumber = 1;
    bool synchronizedStreams = true;
    int segundos = 1;
    int frames = 3;
//    Buffer<Mat> *imageBuffer = new Buffer<Mat>(frames * segundos);
//    sharedImageBuffer = new SharedImageBuffer();
//    sharedImageBuffer->add(deviceNumber, imageBuffer, synchronizedStreams);
//    sharedImageBuffer->setSyncEnabled(true);
    //_circularBuffer = new FIFOBuffer<cv::Mat>(20);
    //_circularBuffer = new FIFOBuffer<cv::Mat>();
    int milliseconds = 22;
    sensorKinect = new KinectSensor(milliseconds);
    connect(sensorKinect, SIGNAL(kinectColorCallback(RGBQUAD*, KinectTime*)),
      this, SLOT(onKinectCallback(RGBQUAD*, KinectTime*)));
    sensorKinect->_deveCapturar = true;
    sensorKinect->start((QThread::Priority)QThread::IdlePriority);

    QThread::sleep(2); //2 segundos
}

void MainWindow::inciarKinectWriter()
{
    //_writerThread = new QThread();
    //_kinectWriter = new KinectWriter(this, _circularBuffer);
    //_kinectWriter->moveToThread(_writerThread);
    //connect(_kinectWriter, SIGNAL(processadoSignal(QString)), ui->lblFPS, SLOT(setText(QString)));
    //connect(_kinectWriter, SIGNAL(iniciarSignal()), _writerThread, SLOT(start()));
    //connect(_writerThread, SIGNAL(started()), _kinectWriter, SLOT(gravarFrame(cv::Mat)));
    //connect(_kinectWriter, SIGNAL(concluidoSignal()), _writerThread, SLOT(quit()), Qt::DirectConnection);
    //_kinectWriter->iniciar();
}

void MainWindow::finalizarKinectWriter()
{
    //_kinectWriter->stop();
    //_writerThread->wait();
    //delete _writerThread;
    //delete _kinectWriter;
}

void MainWindow::deveGravarVideoSlot()
{
//    _deveGravarVideo = ui->chkGravar->isChecked();
//    if (_deveGravarVideo)
//    {
//        inciarKinectWriter();
//    }
//    else
//    {
//        finalizarKinectWriter();
//    }
}

void MainWindow::capturar()
{

    //_deveProcessar = true;


//    KinectWriter *kw = new KinectWriter();
//    kw->moveToThread(&writerThread);

//    connect(kw, &QThread::finished, kw, &QObject::deleteLater);
//    //connect(this, &MainWindow::operate, kw, &KinectWriter::gravar);

//    connect(kw, &KinectWriter::kinectWriterCallBack, this, &MainWindow::onKinectWriterCallback);
//    //connect(writerThread, SIGNAL(destroyed()), kw, SLOT(deleteLater()));
//    writerThread.start();



//    BOOLEAN isSensorConectado = false;
//    HRESULT hr = sensorKinect->get_IsAvailable(&isSensorConectado);
//    if (isSensorConectado == TRUE)
//    {
//        if (_deveGravarVideo == true)
//        {
//            definirTela();
//        }
//        else
//        {
//            ui->btnCapturar->setText("Parar");
//            ui->btnCapturar->setStyleSheet("background-color: red");
//            ui->displayLatencia->display(QTime(0, 0).toString());
//            _deveGravarVideo = true;
//            _cron.reiniciar();
//            _timer.start(1000);
//        }
//    }
}



void MainWindow::onKinectCallback(RGBQUAD *buffer, KinectTime *time)
{
    FPS(ui->lblFPS, time->valor);

    ArenaCena *cena = new ArenaCena(this);
    //QImage image = QImage(buffer, 1900, 1080, QImage::Format_RGB888);
    //QPixmap qPixmap = QPixmap::fromImage(image, Qt::ColorOnly);
    QPixmap qPixmap = QPixmap::fromImage(QImage((uchar*) buffer, _frmWidth_1920, _frmHeight_1080,
      QImage::Format_RGB32).scaledToHeight(ui->viewCaptura->frameSize().height()));
      //QImage::Format_RGB32).scaled(ui->viewCaptura->frameSize()));
    cena->addPixmap(qPixmap);
    //ui->label->setPixmap(QPixmap::fromImage(qimage.scaled(ui->label->frameSize())));
    ui->viewCaptura->setCena(cena);

//    if (_deveGravarVideo == true && _kinectWriter != NULL)
//    {
//        //kinectWriter->finalizar();
//        //writerThread->wait();
//        //kinectWriter->iniciar();

//        cv::Mat img;
//        cv::Mat frmCor(_frmHeight_1080, _frmWidth_1920, CV_8UC4, buffer);
//        //cv::resize(frmCor, img, sz2, 0, 0);
//        cv::cvtColor(frmCor, img, CV_BGRA2BGR);
//        _kinectWriter->gravarFrame(img);
//    }

    //ui->label->setPixmap(QPixmap::fromImage(QImage((uchar*) buffer,
    //  cColorWidth, cColorHeight, QImage::Format_RGB32).scaledToWidth(1280)));
}

void MainWindow::desmarcarFerramentas()
{
    ui->btnArena->setChecked(false);
    ui->btnSelecionar->setChecked(false);
    ui->btnPlataforma->setChecked(false);
}

void MainWindow::salvarArena()
{

}

void MainWindow::toggled()
{
//    if (ui->rdbOriginal->isChecked())
//    {
//        ui->tabViewEsquerda->setCurrentIndex(0);
//        ui->lblPrimeira->setText("HSV");
//        ui->lblSegunda->setText("RGB");
//        ui->lblTerceira->setText("Contraste");
//    }
//    if (ui->rdbHSV->isChecked())
//    {
//        ui->tabViewEsquerda->setCurrentIndex(1);
//        ui->lblPrimeira->setText("Original");
//        ui->lblSegunda->setText("RGB");
//        ui->lblTerceira->setText("Contraste");
//    }
//    else if (ui->rdbRGB->isChecked())
//    {
//        ui->tabViewEsquerda->setCurrentIndex(2);
//        ui->lblPrimeira->setText("Original");
//        ui->lblSegunda->setText("HSV");
//        ui->lblTerceira->setText("Contraste");
//    }
//    else if (ui->rdbMascara->isChecked())
//    {
//        ui->tabViewEsquerda->setCurrentIndex(3);
//        ui->lblPrimeira->setText("Original");
//        ui->lblSegunda->setText("HSV");
//        ui->lblTerceira->setText("RGB");
//    }
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

void MainWindow::posicionarX(int valor)
{
    ui->sliderCirculoX->setMinimum(_raioArena);
    ui->sliderCirculoX->setMaximum(512 - _raioArena);

    if(valor >= _raioArena && valor <= (512 - _raioArena))
    {
       _eixoXArena = valor;
    }
}

void MainWindow::posicionarY(int valor)
{
    ui->sliderCirculoY->setMinimum(_raioArena);
    ui->sliderCirculoY->setMaximum(424 - _raioArena);

    if(valor >= _raioArena && valor <= (424 - _raioArena))// (_raio * 2))
    {
       _eixoYArena = valor;
    }
}

void MainWindow::ajustarContraste(int valor)
{
    contraste = valor;
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {

    }
}

void MainWindow::dimensionarCirculo(int valor)
{
    _raioArena = valor;
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

  //printf("Matrix: %s %dw x %dh \n", str.c_str(), frm.cols, frm.rows);
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

        //ui->viewDeteccao->display(QTime(0, 0).toString());
        _cron.reiniciar();
        _cronPlay.reiniciar();
        _timerPlay.start(33);
    }
}

cv::Point centroDaPlataforma;

void MainWindow::adicionarRegistro(cv::Mat frm)
{
    // Adicionar o registro
    cv::circle(frm, cv::Point(_eixoXArena, _eixoYArena), _raioArena, amarelo, 0.5);
    cv::circle(frm, cv::Point(_eixoXArena, _eixoYArena), _raioArena - 35, branco, 1);

    if (ui->rdbNO->isChecked())
    {
        int espessura = 2;
        int giro = 180;
        int anguloInicial = 0;
        int anguloFinal = 360;

        centroDaPlataforma = cv::Point(_eixoXArena + 100, _eixoYArena + 120);
        cv::Size axes(_raioPlataforma, _raioPlataforma);
        cv::Scalar vermelho(255, 0, 0);
        cv::Scalar verde(0, 255, 0);
        cv::Scalar amarelo(0, 0, 255);
        cv::Scalar branco(255, 255, 255);
        cv::Scalar preto(0, 0, 0);
        cv::ellipse(frm, centroDaPlataforma, axes, giro, anguloInicial, anguloFinal, vermelho, espessura, 8);
    }
    else if (ui->rdbNE->isChecked())
    {
        //NE
        cv::circle(frm, cv::Point((_eixoXArena + (_raioArena / 3)), (_eixoYArena - (_raioArena / 3))),  _raioArena / 9, branco, 2);
    }
    else if (ui->rdbSE->isChecked())
    {
        //SE
        cv::circle(frm, cv::Point((_eixoXArena + (_raioArena / 3)), (_eixoYArena + (_raioArena / 3))),  _raioArena / 9, branco, 2);
    }
    else if (ui->rdbSO->isChecked())
    {
        //SO
        cv::circle(frm, cv::Point((_eixoXArena - (_raioArena / 3)), (_eixoYArena + (_raioArena / 3))),  _raioArena / 9, branco, 2);
    }

    cv::line(frm, cv::Point(_eixoXArena, (_eixoYArena - _raioArena - 20)), cv::Point(_eixoXArena, (_eixoYArena + _raioArena + 20)), branco, 1, cv::LINE_8);
    cv::line(frm, cv::Point((_eixoXArena - _raioArena - 20), _eixoYArena), cv::Point((_eixoXArena + _raioArena + 20), _eixoYArena), branco, 1, cv::LINE_8);
}


void MainWindow::atualizarTela()
{
    // Criar frame colorido
    int _640 = 640;
    int _530 = 530;

    cv::Mat frmCor(424, 512, CV_8UC4, _512x424_cor_buffer);
    //cv::resize(mat, Rect(640,530));
    cv::Mat frmCorRGB, frmMascara, frmMascaraFinal, frmCinza,
            frmCinzaInRange, frmContraste, frmContrasteFinal,
            frmBGR2HSV, frmHSVInRange, frmHSVContraste, frmHSVFinal;

    cv::cvtColor(frmCor, frmCorRGB, cv::COLOR_BGR2RGB);
    cv::cvtColor(frmCor, frmBGR2HSV, cv::COLOR_BGR2HSV);
    cv::cvtColor(frmCor, frmCinza, cv::COLOR_BGR2GRAY);

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
    cv::ellipse(frmCinzaElipse, cv::Point(_eixoXArena, _eixoYArena), cv::Size(_raioArena, _raioArena), 0, 0, 360, branco, -1, 8);
    cv::Mat frmCinzaRedondo(424, 512, CV_8UC1);
    cv::bitwise_and(frmCinzaInRange, frmCinzaElipse, frmCinzaRedondo);

    cv::Mat frmCinzaRedondo2(424, 512, CV_8UC1);
    cv::bitwise_and(frmCinza, frmCinzaElipse, frmCinzaRedondo2);

    cv::Mat frmCinzaRedondo3(424, 512, CV_8UC1);  //frmHSVInRange
    cv::bitwise_and(frmHSVInRange, frmCinzaElipse, frmCinzaRedondo3);

    cv::inRange(frmCinzaRedondo, Scalar(_minContraste), Scalar(_maxContraste), frmMascara);
    cv::Mat frmPreto1(frmCinzaRedondo.size(), CV_8U, Scalar(0));
    frmPreto1.copyTo(frmCinzaRedondo, frmMascara);

    cv::inRange(frmCinzaRedondo2, Scalar(_minContraste), Scalar(_maxContraste), frmContraste);
    cv::Mat frmPreto2(frmCinzaRedondo2.size(), CV_8U, Scalar(0));
    frmPreto2.copyTo(frmCinzaRedondo2, frmContraste);

    cv::inRange(frmCinzaRedondo3, Scalar(_minContraste), Scalar(_maxContraste), frmHSVContraste);
    cv::Mat frmPreto3(frmCinzaRedondo3.size(), CV_8U, Scalar(0));
    frmPreto3.copyTo(frmCinzaRedondo3, frmHSVContraste);

    int morph_elem = 0;
    int morph_size = 2;
    Mat element = getStructuringElement(morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ),
                                        Point( morph_size, morph_size ) );
    cv::morphologyEx(frmHSVContraste, frmHSVContraste, cv::MORPH_CLOSE, element);

    cv::cvtColor(frmHSVContraste, frmHSVFinal, CV_GRAY2BGR);

    int blur = 25;
    cv::GaussianBlur(frmHSVContraste, frmHSVContraste, cv::Size(blur, blur), 0);

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

        if (mx < 0 || my < 0)
        {
            _cronAuto.reiniciar();
        }
        else if (mx > 0 && my > 0)
        {
            // Desenha o centroid.
            cv::Point centroid = cv::Point(mx, my);
            cv::circle(frmHSVFinal, centroid, 2, azul, -1);

            //The distance between ⟨xc,yc⟩ and ⟨xp,yp⟩ is given by the Pythagorean theorem as
            //d=√(xp−xc)2+(yp−yc)2
            //The point ⟨xp,yp⟩ is inside the circle if d<r, on the circle if d=r,
            //and outside the circle if d>r. You can save yourself a little work by comparing
            //d2 with r2 instead: the point is inside the circle if d2<r2, on the circle
            //if d2=r2, and outside the circle if d2>r2. Thus, you want to compare the
            //number (xp−xc)2+(yp−yc)2(xp−xc)2+(yp−yc)2 with r2r2.

            int xSqrt = centroid.x - centroDaPlataforma.x;
            int ySqrt = centroid.y - centroDaPlataforma.y;
            //int d = sqrt(pow(xSqrt, 2) + pow(ySqrt, 2));
            if(sqrt(pow(xSqrt, 2) + pow(ySqrt, 2)) > _raioPlataforma)
            {
                QTime cronAuto = _cronAuto.getTime();
                QTime _3s = QTime(0,0,3,0);

                if (cronAuto.operator >=(_3s) && (_x > 0 && _y > 0))
                {
                    //_cronCalc.reiniciar();
                    //2,14 px por cm;
                    double diametro = _raioArena * 2;
                    double px_por_cm = diametro / double(168);
                    double x = mx - _x;
                    double y = my - _y;
                    double xSqr = x * x;
                    double ySqr = y * y;
                    double distanciaMedida = sqrt(xSqr + ySqr);
                    float FATOR_AJUSTE = 0.9f;
                    distancia_em_cm += round((distanciaMedida / px_por_cm) * FATOR_AJUSTE);

                    cv::Point origem = cv::Point(_x, _y);
                    cv::line(frameLine, centroid, origem, verde, 1.5);
                    cv::add(frameLine, frmRastros, frmRastros);

                    //string strDistancia = "D " + boost::lexical_cast<std::string>(distancia_em_cm) +"cm";
                    ui->displayDistancia->display(distancia_em_cm);
                    QTime tempoCorrido = _cronPlay.getTime();
                    ui->displayLatencia->display(tempoCorrido.toString());
                    //int msegundos = tempoCorrido.elapsed();
                    double segundos = QTime(0, 0, 0).secsTo(tempoCorrido);
                    double velocidade = (distancia_em_cm / 100) / segundos;
                    ui->displayVelocidade->display(QString::number(velocidade, 'f', 2));

                    //cv::putText(frmCorRGB, strDistancia, cv::Point(390,30), cv::FONT_HERSHEY_DUPLEX, 0.7, vermelho);

                    cv::addWeighted(frmCorRGB, 0.7, frmRastros, 1, 1, frmCorRGB);
                    cv::addWeighted(frmHSVFinal, 0.7, frmRastros, 1, 1, frmHSVFinal);
                }
                _x = centroid.x;
                _y = centroid.y;
            }
        }
    }

//    if (ui->rdbOriginal->isChecked())
//    {
        adicionarRegistro(frmCorRGB);
        printType(frmCorRGB);
        ArenaCena *cena = new ArenaCena(this);
        QImage image = QImage(frmCorRGB.data, 512, 424, QImage::Format_RGB888);
        QPixmap qPixmap = QPixmap::fromImage(image, Qt::ColorOnly);

        if (ui->tabViewPrincipal->currentIndex() == 0)
        {
            cena->addPixmap(qPixmap.scaledToWidth(640));
            ui->viewCaptura->setCena(cena);
            //ui->viewArena->scene()->addPixmap(QPixmap::fromImage(image).scaledToWidth(640));
            //ui->viewCaptura->setPixmap(QPixmap::fromImage(image).scaledToWidth(640));
        }
        else if (ui->tabViewPrincipal->currentIndex() == 1)
        {
            cena->addPixmap(qPixmap.scaledToWidth(640));
            ui->viewArena->setCena(cena);
            //QImage qImage((uchar*)_512x424_cor_buffer, 512, 424, QImage::Format_RGB32);

            //QImage qImage( colorImage.data, colorImage.cols, colorImage.rows,
            //              static_cast<int>(colorImage.step), QImage::Format_ARGB32);
           // QPixmap qPixmap = QPixmap::fromImage(image, Qt::ColorOnly);


            //ui->viewArena->scene()->addPixmap(QPixmap::fromImage(image).scaledToWidth(640));
           // ui->viewArena->setPixmap(QPixmap::fromImage(image).scaledToWidth(640));
        }
        else if (ui->tabViewPrincipal->currentIndex() == 2)
        {
            cv::cvtColor(frmHSVContraste, frmHSVContraste, CV_GRAY2RGB);
            QImage img1 = QImage(frmHSVContraste.data, 512, 424, QImage::Format_RGB888);
            //ui->vPrimeira->setPixmap(QPixmap::fromImage(img1).scaledToWidth(256));
            ui->vPrimeira->scene()->addPixmap(QPixmap::fromImage(image).scaledToWidth(256));

            cv::cvtColor(frmMascara, frmMascaraFinal, CV_GRAY2RGB);
            QImage img2 = QImage(frmMascaraFinal.data, 512, 424, QImage::Format_RGB888);
            //ui->vSegunda->setPixmap(QPixmap::fromImage(img2).scaledToWidth(256));
            ui->vSegunda->scene()->addPixmap(QPixmap::fromImage(image).scaledToWidth(256));

            cv::cvtColor(frmContraste, frmContrasteFinal, CV_GRAY2RGB);
            QImage img3 = QImage(frmContrasteFinal.data, 512, 424, QImage::Format_RGB888);
            //ui->vTerceira->setPixmap(QPixmap::fromImage(img3).scaledToWidth(256));
            ui->vTerceira->scene()->addPixmap(QPixmap::fromImage(image).scaledToWidth(256));
        }


//    }

//    if (ui->rdbHSV->isChecked())
//    {
//        //cv::cvtColor(frmHSVContraste, frmHSVFinal, CV_GRAY2BGR);
//        adicionarRegistro(frmHSVFinal);
//        cv::cvtColor(frmHSVFinal,frmHSVFinal,CV_BGR2RGB);
//        QImage image = QImage(frmHSVFinal.data, 512, 424, QImage::Format_RGB888);
//        ui->viewDeteccao->setPixmap(QPixmap::fromImage(image));
//    }

//    if (ui->rdbRGB->isChecked())
//    {
//        cv::cvtColor(frmMascara, frmMascaraFinal, CV_GRAY2BGR);
//        adicionarRegistro(frmMascaraFinal);
//        cv::cvtColor(frmMascaraFinal,frmMascaraFinal, CV_BGR2RGB);
//        QImage image = QImage(frmMascaraFinal.data, 512, 424, QImage::Format_RGB888);
//        ui->viewDeteccao->setPixmap(QPixmap::fromImage(image));
//    }

//    if (ui->rdbMascara->isChecked())
//    {
//        cv::cvtColor(frmContraste, frmContrasteFinal, CV_GRAY2BGR);
//        adicionarRegistro(frmContrasteFinal);
//        cv::cvtColor(frmContrasteFinal, frmContrasteFinal, CV_BGR2RGB);
//        QImage image = QImage(frmContrasteFinal.data, 512, 424, QImage::Format_RGB888);
//        ui->viewDeteccao->setPixmap(QPixmap::fromImage(image));
//    }

    // thumbs
//    if (ui->rdbOriginal->isChecked())
//    {

//    }
//    else if (ui->rdbHSV->isChecked())
//    {
//        QImage img1 = QImage(frmCor.data, 512, 424, QImage::Format_RGB32);
//        ui->vPrimeira->setPixmap(QPixmap::fromImage(img1).scaledToWidth(205));

//        cv::cvtColor(frmMascara, frmMascaraFinal, CV_GRAY2RGB);
//        QImage img2 = QImage(frmMascaraFinal.data, 512, 424, QImage::Format_RGB888);
//        ui->vSegunda->setPixmap(QPixmap::fromImage(img2).scaledToWidth(205));

//        cv::cvtColor(frmContraste, frmContrasteFinal, CV_GRAY2RGB);
//        QImage img3 = QImage(frmContrasteFinal.data, 512, 424, QImage::Format_RGB888);
//        ui->vTerceira->setPixmap(QPixmap::fromImage(img3).scaledToWidth(205));
//    }
//    else if (ui->rdbRGB->isChecked())
//    {
//        QImage img1 = QImage(frmCor.data, 512, 424, QImage::Format_RGB32);
//        ui->vPrimeira->setPixmap(QPixmap::fromImage(img1).scaledToWidth(205));

//        cv::cvtColor(frmHSVContraste, frmHSVContraste, CV_GRAY2RGB);
//        QImage img2 = QImage(frmHSVContraste.data, 512, 424, QImage::Format_RGB888);
//        ui->vSegunda->setPixmap(QPixmap::fromImage(img2).scaledToWidth(205));

//        cv::cvtColor(frmContraste, frmContrasteFinal, CV_GRAY2RGB);
//        QImage img3 = QImage(frmContrasteFinal.data, 512, 424, QImage::Format_RGB888);
//        ui->vTerceira->setPixmap(QPixmap::fromImage(img3).scaledToWidth(205));
//    }
//    else if (ui->rdbMascara->isChecked())
//    {
//        QImage img1 = QImage(frmCor.data, 512, 424, QImage::Format_RGB32);
//        ui->vPrimeira->setPixmap(QPixmap::fromImage(img1).scaledToWidth(205));

//        cv::cvtColor(frmHSVContraste, frmHSVContraste, CV_GRAY2RGB);
//        QImage img2 = QImage(frmHSVContraste.data, 512, 424, QImage::Format_RGB888);
//        ui->vSegunda->setPixmap(QPixmap::fromImage(img2).scaledToWidth(205));

//        cv::cvtColor(frmMascara, frmMascaraFinal, CV_GRAY2RGB);
//        QImage img3 = QImage(frmMascaraFinal.data, 512, 424, QImage::Format_RGB888);
//        ui->vTerceira->setPixmap(QPixmap::fromImage(img3).scaledToWidth(205));
//    }
}

void MainWindow::inverterContraste()
{
    _invertido =  ui->chkInvertido->isChecked();
}

void MainWindow::alterarStatus()
{
//    if(ui->tabViewPrincipal->currentIndex() == 0)
//    {
//       _deveProcessar = true;
//    }
//    else if(ui->tabViewPrincipal->currentIndex() == 1)
//    {
//       _deveProcessar = false;
//    }
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

QThread writerThread;

void MainWindow::onKinectWriterCallback()
{
    printf("Terminou!");
}



void MainWindow::definirTela()
{
    ui->btnCapturar->setText("Gravar");
    ui->btnCapturar->setStyleSheet("background-color: green");
    _deveGravarVideo = false;
    _timer.stop();
}

void MainWindow::atualizarLCD()
{
    ui->displayLatencia->display(_cron.getTime().toString());
    //ui->viewDeteccao->display(_cron.getTime().toString());
    QTime cronTime = _cron.getTime();
    QTime maxTime = ui->cboTimerLimite->time();
    QTime maxTimePlay = ui->cboTimerPlay->time();
    if (_pararGravacaoAuto == true && cronTime.operator >=(maxTime) || cronTime.operator >=(maxTimePlay))
    {
        definirTela();
    }
}

void MainWindow::atualizarDisplayPlay()
{
  //  ui->viewDeteccao->display(_cron.getTime().toString());
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

void MainWindow::processarCor()
{

}

void MainWindow::processarInfrared()
{

}

void MainWindow::processarDepth()
{

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
        label->setStyleSheet("color: white");
    }

    INT64 now = GetTickCount64();
    if (proximaAtualizacaoTime <= now)
    {
        label->setText(QString::number(fps, 'f', 1) + " fps");
        proximaAtualizacaoTime = now + 1000; //1s
        ultimaContagem = qpcNow.QuadPart;
        totalDeframesAtualizados = 0;
    }
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
        sensorKinect->_deveCapturar = false;
        sensorKinect = NULL;
        QThread::msleep(100);
    }

    finalizarKinectWriter();

    //SafeRelease(sensorKinect);
}
