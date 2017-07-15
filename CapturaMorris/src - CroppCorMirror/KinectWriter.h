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
    void openNewFileForWriting(std::string nomeDoArquivo);
	void openCurrentFileForReading();
	
	// leave filename blank if you want the filename to be generated from the date
	void setCurrentFilename(std::string filename = ""); 

    //void writeFrame(std::vector<Point3s> points, std::vector<RGB> colors);
    void gravarFrame(std::string nomeDoArquivo, RGBQUAD* colors, UINT size);
    void writeInfraredFrame(RGBQUAD* infrareds, UINT size);
    void write3DFrame(RGBQUAD* trids, UINT size);

    void writeAllFrame(RGBQUAD* colors, UINT sizeColor,
                       RGBQUAD* infrareds, UINT sizeInfra,
                       RGBQUAD* trids, UINT sizeTrids);
    void writeColorInfraredFrame(RGBQUAD* colors, UINT sizeColor,
                                 RGBQUAD* infrareds, UINT sizeInfra);
    void writeColor3DFrame(RGBQUAD* colors, UINT sizeColor,
                           RGBQUAD* trids, UINT sizeTrids);
    void writeInfra3DFrame(RGBQUAD* infrareds, UINT sizeInfra,
                           RGBQUAD* trids, UINT sizeTrids);

	bool readFrame(std::vector<Point3s> &outPoints, std::vector<RGB> &outColors);

    bool isAbertoParaGravar() { return m_bColorFileOpenedForWriting; }
    bool openedForReading() { return m_bColorFileOpenedForReading; }


    void fecharArquivoSeAberto();

    ~KinectWriter();

private:
	void resetTimer();
	int getRecordingTimeMilliseconds();

    wchar_t* path = L"tracks";

	FILE *m_pFileHandle = nullptr;
    bool m_bColorFileOpenedForWriting = false;
    bool m_bColorFileOpenedForReading = false;

    bool m_bInfraredFileOpenedForWriting = false;
    bool m_bInfraredFileOpenedForReading = false;

    bool m_b3DFileOpenedForWriting = false;
    bool m_b3DFileOpenedForReading = false;

	std::string m_sFilename = "";

	std::chrono::steady_clock::time_point recording_start_time;
};
