#include "KinectReader.h"

KinectReader::KinectReader(std::string nomeDoArquivo, int totalDeBytes, RGBQUAD* frameBuffer)
{
    this->frameBuffer = frameBuffer;
    this->nomeDoArquivo = nomeDoArquivo;
    //this->frameBuffer = new RGBQUAD[totalDeBytes];
}

int count = 0;
void KinectReader::processar()
{
    if (!arquivoAbertoParaLeitura)
    {
        abrirArquivoParaLeitura();
    }

    FILE *arquivo = arquivoHandle;

    do{
        int totalDeBytes, timestamp;
        char tmp[1024];
        int linha = fscanf_s(arquivo, "%s %d %s %d", tmp, 1024, &totalDeBytes, tmp, 1024, &timestamp);

        if (linha < 4)
            return;

        if (totalDeBytes == 0)
            return;

        fgetc(arquivo);
        fread((void*)frameBuffer, totalDeBytes, 1, arquivo);
        fgetc(arquivo);

        printf("Frame lido: %d\n", (++count));
        boost::this_thread::sleep(boost::posix_time::milliseconds(33));

        emit processado();

     } while (!feof(arquivo));

    emit concluido();
}

void KinectReader::abrirArquivoParaLeitura()
{
    fecharArquivoSeAberto();
    arquivoHandle = fopen(nomeDoArquivo.c_str(), "rb");
    arquivoAbertoParaLeitura = true;
}

void KinectReader::fecharArquivoSeAberto()
{
    if (arquivoHandle == nullptr)
    {
        return;
    }
    fclose(arquivoHandle);
    arquivoHandle = nullptr;
    arquivoAbertoParaLeitura = false;
}
