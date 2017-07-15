// Created by Heresy @ 2015/01/16
// Blog Page: http://kheresy.wordpress.com/2015/01/16/k4w-v2-read-color-and-ir-image/
// This sample is used to read infrared image from Kinect v2 sensor, and show the image with OpenCV.

// Standard Library
#include <iostream>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>

using namespace std;

int main(int argc, char** argv)
{
	// 1a. Get default Sensor
	cout << "Try to get default sensor" << endl;
	IKinectSensor* pSensor = nullptr;
	if (GetDefaultKinectSensor(&pSensor) != S_OK)
	{
		cerr << "Get Sensor failed" << endl;
		return -1;
	}

	// 1b. Open sensor
	cout << "Try to open sensor" << endl;
	if (pSensor->Open() != S_OK)
	{
		cerr << "Can't open sensor" << endl;
		return -1;
	}

	// 2a. Get frame source
	cout << "Try to get Infrared source" << endl;
	IInfraredFrameSource* pFrameSource = nullptr;
	if (pSensor->get_InfraredFrameSource(&pFrameSource) != S_OK)
	{
		cerr << "Can't get Infrared frame source" << endl;
		return -1;
	}

	// 2b. Get frame description
	cout << "get Infrared frame description" << endl;
	int		iWidth = 0;
	int		iHeight = 0;
	IFrameDescription* pFrameDescription = nullptr;
	if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
	{
		pFrameDescription->get_Width(&iWidth);
		pFrameDescription->get_Height(&iHeight);
	}
	pFrameDescription->Release();
	pFrameDescription = nullptr;

	// 3a. get frame reader
	cout << "Try to get Infrared frame reader" << endl;
	IInfraredFrameReader* pFrameReader = nullptr;
	if (pFrameSource->OpenReader(&pFrameReader) != S_OK)
	{
		cerr << "Can't get Infrared frame reader" << endl;
		return -1;
	}

	// 2c. release Frame source
	cout << "Release frame source" << endl;
	pFrameSource->Release();
	pFrameSource = nullptr;

	// create OpenCV window
	cv::namedWindow( "Infrared Image" );

	// Enter main loop
	while (true)
	{
		// 4a. Get last frame
		IInfraredFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			// 4c. Copy to OpenCV image
			UINT	uSize = 0;
			UINT16*	pBuffer = nullptr;
			if (pFrame->AccessUnderlyingBuffer(&uSize, &pBuffer) == S_OK)
			{
				cv::Mat mIRImg(iHeight, iWidth, CV_16UC1, pBuffer);
				cv::imshow("Infrared Image", mIRImg);
			}
			else
			{
				cerr << "Data access error" << endl;
			}

			// 4e. release frame
			pFrame->Release();
		}

		// 4f. check keyboard input
		if (cv::waitKey(30) == VK_ESCAPE){
			break;
		}
	}

	// 3b. release frame reader
	cout << "Release frame reader" << endl;
	pFrameReader->Release();
	pFrameReader = nullptr;

	// 1c. Close Sensor
	cout << "close sensor" << endl;
	pSensor->Close();

	// 1d. Release Sensor
	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}
