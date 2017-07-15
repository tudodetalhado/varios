#include <ctime>
#include "KinectWriter.h"

KinectWriter::KinectWriter()
{
}

void KinectWriter::fecharArquivoSeAberto()
{
	if (m_pFileHandle == nullptr)
		return;

	fclose(m_pFileHandle);
	m_pFileHandle = nullptr; 
    arquivoAbertaParaGravar = false;
}

void KinectWriter::resetTimer()
{
	recording_start_time = std::chrono::steady_clock::now();
}

int KinectWriter::getRecordingTimeMilliseconds()
{
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds >(end - recording_start_time).count());
}

void KinectWriter::abrirArquivosParaGravar(std::string nomeDoArquivo)
{
    fecharArquivoSeAberto();

    boost::filesystem::path dir(path);

    if(!(boost::filesystem::exists(dir)))
    {
        boost::filesystem::create_directory(dir);
    }

    if (nomeDoArquivo.empty())
    {
        nomeDoArquivo = "trk";
    }

    char filename[1024];
	time_t t = time(0);
	struct tm * now = localtime(&t);
    sprintf(filename, "%ls\\%s_%04d%02d%02d%02d%02d%02d.knt",
            path,
            nomeDoArquivo.c_str(),
            now->tm_year + 1900,
            now->tm_mon + 1,
            now->tm_mday,
            now->tm_hour,
            now->tm_min,
            now->tm_sec);

    _nomeDoArquivo = filename;
	m_pFileHandle = fopen(filename, "wb");

	resetTimer();
}

int count2 = 0;


void KinectWriter::gravarFrameCv(std::string nomeDoArquivo, cv::Mat frame, UINT size)
{
    if (!arquivoAbertaParaGravar)
    {
        abrirArquivosParaGravar(nomeDoArquivo);
        arquivoAbertaParaGravar = true;
    }

    FILE *fileStream = m_pFileHandle;

    fprintf(fileStream, "n_points= %d\nframe_timestamp= %d\n", size, getRecordingTimeMilliseconds());
    //fwrite((void*)frame, size, 1, fileStream);
    fwrite((void*)frame.data, size, 1, fileStream);
    fprintf(fileStream, "\n");
    printf("Gravado: %d\n", (++count2));
}

bool KinectWriter::writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat)
{

    if(out_mat.empty()){
        int s = 0;
        ofs.write((const char*)(&s), sizeof(int));
        return true;
    }

    int type = out_mat.type();
    ofs.write((const char*)(&out_mat.rows), sizeof(int));
    ofs.write((const char*)(&out_mat.cols), sizeof(int));
    ofs.write((const char*)(&type), sizeof(int));
    ofs.write((const char*)(out_mat.data), out_mat.elemSize() * out_mat.total());

    return true;
}


//! Save cv::Mat as binary
/*!
\param[in] filename filaname to save
\param[in] output cvmat to save
*/
bool KinectWriter::SaveMatBinary(std::string& filename, const cv::Mat& output)
{
//    if (!arquivoAbertaParaGravar)
//    {
//
//        arquivoAbertaParaGravar = true;
//    }

    boost::filesystem::path dir(path);

    if(!(boost::filesystem::exists(dir)))
    {
        boost::filesystem::create_directory(dir);
    }

    if (filename.empty())
    {
        filename = "trk";
    }

    char filenameFinal[1024];
    time_t t = time(0);
    struct tm * now = localtime(&t);
    sprintf(filenameFinal, "%ls\\%s_%04d%02d%02d%02d%02d%02d.knt",
            path,
            filename.c_str(),
            now->tm_year + 1900,
            now->tm_mon + 1,
            now->tm_mday,
            now->tm_hour,
            now->tm_min,
            now->tm_sec);

    //const char* str = filenameFinal.c_str();
    //std::ofstream ofs(filenameFinal, std::ios::binary);
//    if(!ofs.is_open()){
//        return false;
//    }

    std::ofstream ofs(filenameFinal, std::ios::binary);
    if (!arquivoAbertaParaGravar)
    {
        //ofs = ofsFunction;
        arquivoAbertaParaGravar = true;
    }
    return writeMatBinary(ofs, output);
}

void KinectWriter::gravarFrame(std::string nomeDoArquivo, RGBQUAD* colorBuffer, UINT size)
{
    if (!arquivoAbertaParaGravar)
    {
        abrirArquivosParaGravar(nomeDoArquivo);
        arquivoAbertaParaGravar = true;
    }

    FILE *fileStream = m_pFileHandle;

    fprintf(fileStream, "n_points= %d\nframe_timestamp= %d\n", size, getRecordingTimeMilliseconds());
    fwrite((void*)colorBuffer, size, 1, fileStream);
    fprintf(fileStream, "\n");
    printf("Gravado: %d\n", (++count2));
}

KinectWriter::~KinectWriter()
{
    fecharArquivoSeAberto();
}
