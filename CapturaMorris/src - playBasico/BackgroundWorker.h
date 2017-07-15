#ifndef BACKGROUNDWORKER_H
#define BACKGROUNDWORKER_H

#include "stdafx.h"
#include <fstream>
#include "MainWindow.h"
#include "BackgroundWorker.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

class BackgroundWorker : public QObject
{
    Q_OBJECT

public:
    BackgroundWorker(std::string nomeDoArquivo, int totalDeBytes, RGBQUAD* frameBuffer);

public slots:
    void processar();

signals:
    void processado();
    void concluido();

private:
    void abrirArquivoParaLeitura();
    void fecharArquivoSeAberto();

    int totalDeBytes;
    RGBQUAD* frameBuffer;
    std::string nomeDoArquivo;
    FILE* arquivoHandle = nullptr;
    bool arquivoAbertoParaLeitura = false;
};

#endif // BACKGROUNDWORKER_H
