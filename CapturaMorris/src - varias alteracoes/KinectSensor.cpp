//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }
#include "KinectSensor.h"
#include <chrono>

KinectSensor::KinectSensor()
{
    kinectSensor = NULL;
    pCoordinateMapper = NULL;
    multiSourceFrameReader = NULL;
    bufferDeProfundidade = NULL;
    bufferDeCor = NULL;
}

KinectSensor::~KinectSensor()
{
    SafeRelease(kinectSensor);
    SafeRelease(pCoordinateMapper);
    SafeRelease(multiSourceFrameReader);
}

bool KinectSensor::inicializar()
{
	HRESULT hr;

    hr = GetDefaultKinectSensor(&kinectSensor);

	if (FAILED(hr))
	{
        inicializado = false;
        return inicializado;
	}

    if (kinectSensor)
	{
        kinectSensor->get_CoordinateMapper(&pCoordinateMapper);
        hr = kinectSensor->Open();

		if (SUCCEEDED(hr))
		{
            kinectSensor->OpenMultiSourceFrameReader(
                FrameSourceTypes::FrameSourceTypes_Color |
                FrameSourceTypes::FrameSourceTypes_Depth |
                FrameSourceTypes::FrameSourceTypes_Infrared,
                &multiSourceFrameReader);
		}
	}

    inicializado = SUCCEEDED(hr);

//    if (inicializado)
//	{
//		std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
//		bool bTemp;
//		do
//		{
//            bTemp = capturarFrame();
			
//			std::chrono::duration<double> elapsedSeconds = std::chrono::system_clock::now() - start;
//			if (elapsedSeconds.count() > 5.0)
//			{
//                inicializado = false;
//				break;
//			}

//		} while (!bTemp);
//	}

    return inicializado;
}

//bool KinectSensor::capturarFrame()
//{
//    if (!inicializado)
//	{
//		return false;
//	}

//	//Multi frame
//	IMultiSourceFrame* pMultiFrame = NULL;
//    HRESULT hr = multiSourceFrameReader->AcquireLatestFrame(&pMultiFrame);

//	if (!SUCCEEDED(hr))
//	{
//		return false;
//	}

//    getDepthFrame(pMultiFrame);
//    getColorFrame(pMultiFrame);

//	SafeRelease(pMultiFrame);

//    bool frameCapturado = false;
//    if (bufferDeProfundidade != NULL && bufferDeCor != NULL)
//    {
//        frameCapturado = true;
//    }

//    return frameCapturado;
//}


UINT16* KinectSensor::getDepthFrame()
{
    //Multi frame
    IMultiSourceFrame* multiFrame = NULL;
    HRESULT hr = multiSourceFrameReader->AcquireLatestFrame(&multiFrame);

    if (!SUCCEEDED(hr))
    {
        return NULL;
    }

    IDepthFrameReference* depthFrameReference = NULL;
    IDepthFrame* depthFrame = NULL;
    multiFrame->get_DepthFrameReference(&depthFrameReference);
    hr = depthFrameReference->AcquireFrame(&depthFrame);

    if (SUCCEEDED(hr))
    {
        if (bufferDeProfundidade == NULL)
        {
            IFrameDescription* frameDescription = NULL;
            hr = depthFrame->get_FrameDescription(&frameDescription);
            frameDescription->get_Width(&depthFrameWidth);
            frameDescription->get_Height(&depthFrameHeight);
            bufferDeProfundidade = new UINT16[depthFrameHeight * depthFrameWidth];
            depthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
            depthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
            SafeRelease(frameDescription);
        }

        UINT bufferSize = depthFrameHeight * depthFrameWidth;
        //hr = depthFrame->CopyFrameDataToArray(bufferSize, bufferDeProfundidade);
        hr = depthFrame->AccessUnderlyingBuffer(&bufferSize, &bufferDeProfundidade);
    }

    SafeRelease(multiFrame);
    SafeRelease(depthFrame);
    SafeRelease(depthFrameReference);

    return bufferDeProfundidade;
}

//UINT16* KinectSensor::getDepthFrame()
//{
//    //Multi frame
//    IMultiSourceFrame* multiFrame = NULL;
//    HRESULT hr = multiSourceFrameReader->AcquireLatestFrame(&multiFrame);

//    if (!SUCCEEDED(hr))
//    {
//        return NULL;
//    }

//    IDepthFrameReference* depthFrameReference = NULL;
//    IDepthFrame* depthFrame = NULL;
//    multiFrame->get_DepthFrameReference(&depthFrameReference);
//    hr = depthFrameReference->AcquireFrame(&depthFrame);

//    if (SUCCEEDED(hr))
//    {
//        if (bufferDeProfundidade == NULL)
//        {
//            IFrameDescription* frameDescription = NULL;
//            hr = depthFrame->get_FrameDescription(&frameDescription);
//            frameDescription->get_Width(&depthFrameWidth);
//            frameDescription->get_Height(&depthFrameHeight);
//            bufferDeProfundidade = new UINT16[depthFrameHeight * depthFrameWidth];
//            depthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
//            depthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
//            SafeRelease(frameDescription);
//        }

//        UINT bufferSize = depthFrameHeight * depthFrameWidth;
//        //hr = depthFrame->CopyFrameDataToArray(bufferSize, bufferDeProfundidade);
//        hr = depthFrame->AccessUnderlyingBuffer(&bufferSize, &bufferDeProfundidade);
//    }

//    SafeRelease(multiFrame);
//    SafeRelease(depthFrame);
//    SafeRelease(depthFrameReference);

//    return bufferDeProfundidade;
//}

RGB* KinectSensor::getColorFrame()
{
    //Multi frame
    IMultiSourceFrame* multiFrame = NULL;
    HRESULT hr = multiSourceFrameReader->AcquireLatestFrame(&multiFrame);

    if (!SUCCEEDED(hr))
    {
        return NULL;
    }

    IColorFrameReference* colorFrameReference = NULL;
    IColorFrame* colorFrame = NULL;
    multiFrame->get_ColorFrameReference(&colorFrameReference);
    hr = colorFrameReference->AcquireFrame(&colorFrame);

    if (SUCCEEDED(hr))
    {
        if (bufferDeCor == NULL)
        {
            IFrameDescription* pFrameDescription = NULL;
            hr = colorFrame->get_FrameDescription(&pFrameDescription);
            hr = pFrameDescription->get_Width(&colorFrameWidth);
            hr = pFrameDescription->get_Height(&colorFrameHeight);
            bufferDeCor = new RGB[colorFrameWidth * colorFrameHeight];
            SafeRelease(pFrameDescription);
        }

        UINT nBufferSize = colorFrameWidth * colorFrameHeight * sizeof(RGB);
        hr = colorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(bufferDeCor), ColorImageFormat_Bgra);
    }

    SafeRelease(multiFrame);
    SafeRelease(colorFrame);
    SafeRelease(colorFrameReference);

    return bufferDeCor;
}

UINT16* KinectSensor::getInfraredFrame()
{
    //Multi frame
    IMultiSourceFrame* multiFrame = NULL;
    HRESULT hr = multiSourceFrameReader->AcquireLatestFrame(&multiFrame);

    if (!SUCCEEDED(hr))
    {
        return NULL;
    }

    IInfraredFrameReference* infraredFrameReference = NULL;
    IInfraredFrame* pInfraredFrame = NULL;;
    multiFrame->get_InfraredFrameReference(&infraredFrameReference);
    hr = infraredFrameReference->AcquireFrame(&pInfraredFrame);
    UINT16 *pBuffer = NULL;

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        UINT nBufferSize = 0;


        hr = pInfraredFrame->get_RelativeTime(&nTime);

        if (SUCCEEDED(hr))
        {
            hr = pInfraredFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Width(&nWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Height(&nHeight);
        }

        if (SUCCEEDED(hr))
        {
            //hr = pInfraredFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
            hr = pInfraredFrame->CopyFrameDataToArray(nBufferSize,pBuffer);
            //CopyFrameDataToArray(nHeight * nWidth, irBuffer);
        }

        SafeRelease(pFrameDescription);
    }

    SafeRelease(multiFrame);
    SafeRelease(pInfraredFrame);
    SafeRelease(infraredFrameReference);

    return pBuffer;
}

void KinectSensor::MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
    pCoordinateMapper->MapDepthFrameToCameraSpace(depthFrameWidth * depthFrameHeight, bufferDeProfundidade, depthFrameWidth * depthFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
}

void KinectSensor::MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
    pCoordinateMapper->MapColorFrameToCameraSpace(depthFrameWidth * depthFrameHeight, bufferDeProfundidade, colorFrameWidth * colorFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
}

void KinectSensor::MapDepthFrameToColorSpace(Point2f *pColorSpacePoints)
{
    pCoordinateMapper->MapDepthFrameToColorSpace(depthFrameWidth * depthFrameHeight, bufferDeProfundidade, depthFrameWidth * depthFrameHeight, (ColorSpacePoint*)pColorSpacePoints);
}

void KinectSensor::MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints)
{
    pCoordinateMapper->MapColorFrameToDepthSpace(depthFrameWidth * depthFrameHeight, bufferDeProfundidade, colorFrameWidth * colorFrameHeight, (DepthSpacePoint*)pDepthSpacePoints);;
}
