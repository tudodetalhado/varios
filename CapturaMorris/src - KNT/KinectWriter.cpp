#include <ctime>
#include "KinectWriter.h"

KinectWriter::KinectWriter()
{
}

void KinectWriter::closeFileIfOpened()
{
	if (m_pFileHandle == nullptr)
		return;

	fclose(m_pFileHandle);
	m_pFileHandle = nullptr; 
    m_bColorFileOpenedForReading = false;
    m_bColorFileOpenedForWriting = false;

    m_bInfraredFileOpenedForReading = false;
    m_bInfraredFileOpenedForWriting = false;

    m_b3DFileOpenedForReading = false;
    m_b3DFileOpenedForWriting = false;
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

void KinectWriter::openCurrentFileForReading()
{
	closeFileIfOpened();

	m_pFileHandle = fopen(m_sFilename.c_str(), "rb");

    m_bColorFileOpenedForReading = true;
    m_bColorFileOpenedForWriting = false;

    m_bInfraredFileOpenedForReading = true;
    m_bInfraredFileOpenedForWriting = false;

    m_b3DFileOpenedForReading = true;
    m_b3DFileOpenedForWriting = false;
}

void KinectWriter::openNewFileForWriting(std::string nomeDoArquivo)
{
	closeFileIfOpened();

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

    m_sFilename = filename;
	m_pFileHandle = fopen(filename, "wb");

	resetTimer();
}

bool KinectWriter::readFrame(std::vector<Point3s> &outPoints, std::vector<RGB> &outColors)
{
    if (!m_bColorFileOpenedForReading)
		openCurrentFileForReading();

	outPoints.clear();
	outColors.clear();
	FILE *f = m_pFileHandle;
	int nPoints, timestamp; 
	char tmp[1024]; 
	int nread = fscanf_s(f, "%s %d %s %d", tmp, 1024, &nPoints, tmp, 1024, &timestamp);

	if (nread < 4)
		return false;

	if (nPoints == 0)
		return true;

	fgetc(f);		//  '\n'
	outPoints.resize(nPoints);
	outColors.resize(nPoints);

	fread((void*)outPoints.data(), sizeof(outPoints[0]), nPoints, f);
	fread((void*)outColors.data(), sizeof(outColors[0]), nPoints, f);
	fgetc(f);		// '\n'
	return true;

}

int count2 = 0;

void KinectWriter::writeColorFrame(std::string nomeDoArquivo, RGBQUAD* colorBuffer, UINT size)
{
    if (!m_bColorFileOpenedForWriting)
    {
        openNewFileForWriting(nomeDoArquivo);
        m_bColorFileOpenedForReading = false;
        m_bColorFileOpenedForWriting = true;
    }

    FILE *fileStream = m_pFileHandle;

    //fprintf_s( stream, "%s %ld %f%c", "a-string", 65000, 3.14159, 'x' );

    //int nPoints = static_cast<int>(points.size());

    //RGBQUAD is 4 byte size;

    //int rgbSize = sizeof(colors);
    fprintf(fileStream, "n_points= %d\nframe_timestamp= %d\n", size, getRecordingTimeMilliseconds());
    //if (nPoints > 0)
    //{
        //fwrite((void*)points.data(), sizeof(points[0]), nPoints, f);
               //buffer, size, elementos, fileStream
    fwrite((void*)colorBuffer, size, 1, fileStream); //sizeof(colors[0])
    //}
    fprintf(fileStream, "\n");
    printf("Gravado: %d\n", (++count2));
}

void KinectWriter::writeInfraredFrame(RGBQUAD* infrareds, UINT size)
{
    if (!m_bInfraredFileOpenedForWriting)
    {
        wchar_t* ext= L"knt";
        //openNewFileForWriting(ext);
        m_bInfraredFileOpenedForReading = false;
        m_bInfraredFileOpenedForWriting = true;
    }

    FILE *fileStream = m_pFileHandle;

    //int nPoints = static_cast<int>(points.size());
    //fprintf(f, "n_points= %d\nframe_timestamp= %d\n", nPoints, getRecordingTimeMilliseconds());
    //if (nPoints > 0)
    //{
        //fwrite((void*)points.data(), sizeof(points[0]), nPoints, f);
               //buffer, size, elementos, fileStream
        fwrite((void*)infrareds, size, sizeof(infrareds[0]), fileStream);
    //}
    fprintf(fileStream, "\n");
}

void KinectWriter::write3DFrame(RGBQUAD* trids, UINT size)
{
    if (!m_b3DFileOpenedForWriting)
    {
        wchar_t* ext= L"knt";
      //  openNewFileForWriting(ext);
        m_b3DFileOpenedForReading = false;
        m_b3DFileOpenedForWriting = true;
    }

    FILE *fileStream = m_pFileHandle;

    //int nPoints = static_cast<int>(points.size());
    //fprintf(f, "n_points= %d\nframe_timestamp= %d\n", nPoints, getRecordingTimeMilliseconds());
    //if (nPoints > 0)
    //{
        //fwrite((void*)points.data(), sizeof(points[0]), nPoints, f);
               //buffer, size, elementos, fileStream
        fwrite((void*)trids, size, sizeof(trids[0]), fileStream);
    //}
    fprintf(fileStream, "\n");
}


void KinectWriter::writeAllFrame(RGBQUAD* colors, UINT sizeColor,
                   RGBQUAD* infrareds, UINT sizeInfra,
                   RGBQUAD* trids, UINT sizeTrids)
{
    wchar_t* ext= L"knt";
    //openNewFileForWriting(ext);

}

void KinectWriter::writeColorInfraredFrame(RGBQUAD* colors, UINT sizeColor,
                             RGBQUAD* infrareds, UINT sizeInfra)
{
    wchar_t* ext= L"knt";
    //openNewFileForWriting(ext);
}

void KinectWriter::writeColor3DFrame(RGBQUAD* colors, UINT sizeColor,
                       RGBQUAD* trids, UINT sizeTrids)
{
    wchar_t* ext= L"knt";
    //openNewFileForWriting(ext);
}

void KinectWriter::writeInfra3DFrame(RGBQUAD* infrareds, UINT sizeInfra,
                       RGBQUAD* trids, UINT sizeTrids)
{
    wchar_t* ext= L"knt";
    //openNewFileForWriting(ext);
}

//void KinectWriter::writeFrame(std::vector<Point3s> points, std::vector<RGB> colors)
//{
//	if (!m_bFileOpenedForWriting)
//		openNewFileForWriting();

//	FILE *f = m_pFileHandle;

//	int nPoints = static_cast<int>(points.size());
//	fprintf(f, "n_points= %d\nframe_timestamp= %d\n", nPoints, getRecordingTimeMilliseconds());
//	if (nPoints > 0)
//	{
//		fwrite((void*)points.data(), sizeof(points[0]), nPoints, f);
//		fwrite((void*)colors.data(), sizeof(colors[0]), nPoints, f);
//	}
//	fprintf(f, "\n");
//}

KinectWriter::~KinectWriter()
{
	closeFileIfOpened();
}
