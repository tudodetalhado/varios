#pragma once

#include <stdio.h>
#include <string>
#include <vector>
#include <chrono>
#include "Utils.h"
#include <boost/filesystem.hpp>

class KinectWriter
{
public:
    KinectWriter();
    void abrirArquivosParaGravar(std::string nomeDoArquivo);

	// leave filename blank if you want the filename to be generated from the date
	void setCurrentFilename(std::string filename = ""); 

    //void writeFrame(std::vector<Point3s> points, std::vector<RGB> colors);
    void gravarFrame(std::string nomeDoArquivo, RGBQUAD* colors, UINT size);

    bool isAbertoParaGravar() { return arquivoAbertaParaGravar; }
    void fecharArquivoSeAberto();

    ~KinectWriter();

private:
	void resetTimer();
	int getRecordingTimeMilliseconds();

    wchar_t* path = L"tracks";

	FILE *m_pFileHandle = nullptr;
    bool arquivoAbertaParaGravar = false;

    std::string _nomeDoArquivo = "";

	std::chrono::steady_clock::time_point recording_start_time;
};
