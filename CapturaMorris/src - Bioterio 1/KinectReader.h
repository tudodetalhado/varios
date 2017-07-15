#ifndef KINECTREADER_H
#define KINECTREADER_H

#include "stdafx.h"
#include <fstream>
#include "MainWindow.h"
#include "KinectReader.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

class KinectReader : public QObject
{
    Q_OBJECT

public:
    KinectReader(std::string nomeDoArquivo, int totalDeBytes, RGBQUAD* frameBuffer);
    //KinectReader(std::string nomeDoArquivo, int totalDeBytes, cv::Mat* frameBuffer);

public slots:
    void processar();

signals:
    void processado();
    void concluido();

private:
    void abrirArquivoParaLeitura();
    void fecharArquivoSeAberto();
    bool readMatBinary(std::ifstream& ifs, cv::Mat& in_mat);
    bool LoadMatBinary(std::string& filename, cv::Mat& output);

    int totalDeBytes;
    RGBQUAD* frameBuffer;
    //cv::Mat* frameBuffer;
    std::string nomeDoArquivo;
    FILE* arquivoHandle = nullptr;
    bool arquivoAbertoParaLeitura = false;
};

#endif // KINECTREADER_H
