// Created by Heresy @ 2015/01/13
// Blog Page: http://kheresy.wordpress.com/2015/01/13/k4w-v2-opencv-depth-viewer/
// This sample is used to read depth map from Kinect v2 sensor, and show the image with OpenCV.

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
	}
	else
	{
		// 1b. Open sensor
		cout << "Try to open sensor" << endl;
		if (pSensor->Open() != S_OK)
		{
			cerr << "Can't open sensor" << endl;
		}
		else
		{
			// 2a. Get frame source
			cout << "Try to get source" << endl;
			IDepthFrameSource* pFrameSource = nullptr;
			if (pSensor->get_DepthFrameSource(&pFrameSource) != S_OK)
			{
				cerr << "Can't get frame source" << endl;
			}
			else
			{
				// 2b. Get frame description
				int		iWidth = 0;
				int		iHeight = 0;
				IFrameDescription* pFrameDescription = nullptr;
				if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
				{
					pFrameDescription->get_Width(&iWidth);
					pFrameDescription->get_Height(&iHeight);
					pFrameDescription->Release();
					pFrameDescription = nullptr;
				}

				// 2c. get some dpeth only meta
				UINT16 uDepthMin = 0, uDepthMax = 0;
				pFrameSource->get_DepthMinReliableDistance(&uDepthMin);
				pFrameSource->get_DepthMaxReliableDistance(&uDepthMax);
				cout << "Reliable Distance: " << uDepthMin << " - " << uDepthMax << endl;

				// perpare OpenCV
				cv::Mat mDepthImg(iHeight, iWidth, CV_16UC1);
				cv::Mat mImg8bit(iHeight, iWidth, CV_8UC1);
				cv::namedWindow( "Depth Map" );

				// 3a. get frame reader
				cout << "Try to get frame reader" << endl;
				IDepthFrameReader* pFrameReader = nullptr;
				if (pFrameSource->OpenReader(&pFrameReader) != S_OK)
				{
					cerr << "Can't get frame reader" << endl;
				}
				else
				{
					// Enter main loop
					cout << "Enter main loop" << endl;
					while (true)
					{
						// 4a. Get last frame
						IDepthFrame* pFrame = nullptr;
						if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
						{
							// 4c. copy the depth map to image
							if (pFrame->CopyFrameDataToArray(iWidth * iHeight, reinterpret_cast<UINT16*>(mDepthImg.data)) == S_OK)
							{
								// 4d. convert from 16bit to 8bit
								mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / uDepthMax);
								cv::imshow("Depth Map", mImg8bit);
							}
							else
							{
								cerr << "Data copy error" << endl;
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
				}

				// 2d. release Frame source
				cout << "Release frame source" << endl;
				pFrameSource->Release();
				pFrameSource = nullptr;
			}

			// 1c. Close Sensor
			cout << "close sensor" << endl;
			pSensor->Close();
		}

		// 1d. Release Sensor
		cout << "Release sensor" << endl;
		pSensor->Release();
		pSensor = nullptr;
	}
	return 0;
}
