#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QTimer>
#include <QFileDialog>
#include <vector>

#pragma warning(push)
#pragma warning(disable:6255)
#pragma warning(disable:6263)
#pragma warning(disable:4995)
#include "ppl.h"
#pragma warning(pop)

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->lcdDisplay->display(QTime(0, 0).toString());
    ui->lcdDisplayPlay->display(QTime(0, 0).toString());
    ui->lblFPS->setStyleSheet("color: red");

    cv::setUseOptimized(true);

    raio = cv::Size(156,156);

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
    connect(ui->btnGravar, SIGNAL(clicked(bool)), this, SLOT(gravarTrack()));
    connect(ui->chkPararAuto, SIGNAL(clicked(bool)), this, SLOT(setPararGravacaoAuto()));
    connect(ui->tabViews, SIGNAL(currentChanged(int)), this, SLOT(alterarStatus()));
    connect(ui->sliderCirculo, SIGNAL(valueChanged(int)), this, SLOT(dimensionarCirculo(int)));
    connect(ui->sliderContraste, SIGNAL(valueChanged(int)), this, SLOT(ajustarContraste(int)));
    connect(ui->chkInvertido, SIGNAL(clicked(bool)), this, SLOT(inverterContraste()));
    connect(ui->chkTrack, SIGNAL(clicked(bool)), this, SLOT(quantificar()));

    // Wait a Few Seconds until begins to Retrieve Data from Sensor ( about 2000-[ms] )
    std::this_thread::sleep_for(std::chrono::seconds(2));
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

void MainWindow::ajustarContraste(int valor)
{
    contraste = valor;
}

void MainWindow::dimensionarCirculo(int valor)
{
    raio = cv::Size(valor,valor);
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
    ui->lcdDisplay->display(_cron.getTime().toString());
    ui->lcdDisplayPlay->display(_cron.getTime().toString());
    QTime cronTime = _cron.getTime();
    QTime maxTime = ui->cboTimerLimite->time();
    QTime maxTimePlay = ui->cboTimerPlay->time();
    if (_pararGravacaoAuto == true && cronTime.operator >=(maxTime) || cronTime.operator >=(maxTimePlay)){
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
            ui->btnGravar->setText("Parar");
            ui->btnGravar->setStyleSheet("background-color: red");
            ui->lcdDisplay->display(QTime(0, 0).toString());
            _deveGravar = true;
            _cron.reiniciar();
            _timer.start(1000);
        }
    }
}

void MainWindow::definirTela()
{
    ui->btnGravar->setText("Gravar");
    ui->btnGravar->setStyleSheet("background-color: green");
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

void MainWindow::atualizarTela()
{
    INT64 _playTime = _cronPlay.getTime().msec();
    FPS(ui->lblFPSPlay, _playTime);
    _cronPlay.reiniciar();

    ui->lblViewPlay->setPixmap(QPixmap::fromImage(QImage((uchar*)
        _512x424_cor_buffer, 512, 424, QImage::Format_RGB32)));
}

//void MainWindow::abrirTrack()
//{
//    QString filtros = " Kinect Stream (*.knt);;"
//                      " Todos (*.*)";

//    QString nomeDoArquivo = QFileDialog::getOpenFileName(
//                this,
//                tr("Abrir arquivo"),
//                QDir::currentPath() + "\\tracks",
//                filtros);

//    if (!nomeDoArquivo.isEmpty())
//    {
//        nomeArquivoAtual = nomeDoArquivo;

//        //int totalDeBytes = 424 * 424 * sizeof(RGBQUAD);
//        UINT totalDeBytes = 424 * 424 * 4 * sizeof(BYTE);

//        QThread *thread;
//        thread = new QThread;
//        KinectReader *kinectReader;
//        std::string nome = nomeArquivoAtual.toUtf8().constData();
//        kinectReader = new KinectReader(nome, totalDeBytes,  &frameColor);
//        kinectReader->moveToThread(thread);
//        connect(thread, SIGNAL(started()), kinectReader, SLOT(processar()));
//        connect(kinectReader, SIGNAL(processado()), this, SLOT(atualizarTela()));
//        connect(kinectReader, SIGNAL(concluido()), thread, SLOT(quit()));
//        thread->start();
//    }
//}

void MainWindow::abrirTrack()
{
    QString filtros = " Kinect Stream (*.knt);;"
                      " Todos (*.*)";

    QString nomeDoArquivo = QFileDialog::getOpenFileName(
                this,
                tr("Abrir arquivo"),
                QDir::currentPath() + "\\tracks",
                filtros);

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
            QString nomeDoArquivo = ui->txtId->toPlainText();
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
  return QImage(mat.data, mat.cols, mat.rows, mat.step, format);
}

cv::Mat lambda(2, 4, CV_32FC1);
std::vector<std::vector<cv::Point>> tetragons;
//int THRESHOLD_ANIMAL_VS_FLOOR = 240;
int THRESHOLD_WALL_VS_FLOOR = 80;
cv::RNG rng(12345);
int _x, _y;
cv::Mat frmRastros = cv::Mat::zeros(424, 512, CV_8UC4);
cv::Mat frameLine = cv::Mat::zeros(424, 512, CV_8UC4);

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
        ui->lblViewOriginal->setPixmap(QPixmap::fromImage(image).scaledToWidth(205));

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
        cv::ellipse(frameMask, centro, raio, 0, 0, 360, branco, -1, 8);
        cv::bitwise_and(frmEspelhado, frameMask, frameRedondo);

        //frame contraste
        cv::cvtColor(frameRedondo, frmCinza, cv::COLOR_BGR2GRAY);

        QImage imgCinza = mat_to_qimage_ref(frmCinza, QImage::Format_Grayscale8);
        ui->lblViewCinza->setPixmap(QPixmap::fromImage(imgCinza).scaledToWidth(205));

        cv::GaussianBlur(frmCinza, frmDesfoco, cv::Size(25,25), 0);

        //(const Mat& src, Mat& dst, double thresh, double maxVal, int thresholdType)
        //src – Source array (single-channel, 8-bit of 32-bit floating point)
        //dst – Destination array; will have the same size and the same type as src
        //thresh – Threshold value
        //maxVal – Maximum value to use with THRESH_BINARY and THRESH_BINARY_INV thresholding types
        //thresholdType – Thresholding type (see the discussion)
        cv::threshold(frmDesfoco, frmConstraste, contraste, 255, threshold);

        QImage imgContraste = mat_to_qimage_ref(frmConstraste, QImage::Format_Grayscale8);
        ui->lblViewContraste->setPixmap(QPixmap::fromImage(imgContraste).scaledToWidth(205));

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
        ui->lblViewRastreamento->setPixmap(QPixmap::fromImage(imgRastros).scaledToWidth(205));
    }
    //FIM DO RASTREAMENTO--------------------------------

    ui->lblViewResultado->setPixmap(QPixmap::fromImage(QImage((uchar*)
        _512x424_cor_buffer, 512, 424, QImage::Format_RGB32)));
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

    ui->lblViewResultado->setPixmap(QPixmap::fromImage(QImage((uchar*)
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

    ui->lblViewResultado->setPixmap(QPixmap::fromImage(QImage((uchar*)
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
