#include "KinectReader.h"

KinectReader::KinectReader(std::string nomeDoArquivo, int totalDeBytes, RGBQUAD* frameBuffer)
//KinectReader::KinectReader(std::string nomeDoArquivo, int totalDeBytes, cv::Mat* frameBuffer)
{
    this->frameBuffer = frameBuffer;
    this->nomeDoArquivo = nomeDoArquivo;
    //this->frameBuffer = new RGBQUAD[totalDeBytes];
}


//! Read cv::Mat from binary
/*!
\param[in] ifs input file stream
\param[out] in_mat mat to load
*/
bool KinectReader::readMatBinary(std::ifstream& ifs, cv::Mat& in_mat)
{
    if(!ifs.is_open()){
        return false;
    }

    int rows, cols, type;
    ifs.read((char*)(&rows), sizeof(int));
    if(rows==0){
        return true;
    }
    ifs.read((char*)(&cols), sizeof(int));
    ifs.read((char*)(&type), sizeof(int));

    in_mat.release();
    in_mat.create(rows, cols, type);
    ifs.read((char*)(in_mat.data), in_mat.elemSize() * in_mat.total());

    return true;
}


//! Load cv::Mat as binary
/*!
\param[in] filename filaname to load
\param[out] output loaded cv::Mat
*/
bool KinectReader::LoadMatBinary(std::string& filename, cv::Mat& output){
    std::ifstream ifs(filename, std::ios::binary);
    return readMatBinary(ifs, output);
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
        boost::this_thread::sleep(boost::posix_time::milliseconds(47));

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
