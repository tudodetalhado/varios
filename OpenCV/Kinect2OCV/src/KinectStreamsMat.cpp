#include "KinectStreamsMat.hpp"


namespace K2OCV 
{
	////////////////////////////////////////////////////////////////////////////////////////////////////
	cv::Mat CKinectStreamsMat::getColorFrame(IColorFrameReader * _color_reader, HRESULT sourceInitHresult)
	{
		HRESULT hr = sourceInitHresult;
		IColorFrame* frame = NULL;
		IFrameDescription* frameDesc;
		cv::Mat colorImage;
		if (SUCCEEDED(hr)) {
			hr = _color_reader->AcquireLatestFrame(&frame);
			if (SUCCEEDED(hr)) {
				hr = frame->get_FrameDescription(&frameDesc);
				if (SUCCEEDED(hr)) {
					int frameWidth = 0, frameHeight = 0;
					hr = frameDesc->get_Width(&frameWidth);
					if (SUCCEEDED(hr)) {
						hr = frameDesc->get_Height(&frameHeight);
					}
					if (SUCCEEDED(hr)) {
						const int imgSize = frameWidth*frameHeight * 4; //4 Channels(BGRA)
						//BYTE* frameData = new BYTE[imgSize];
						colorImage = cv::Mat(frameHeight, frameWidth, CV_8UC4);
						hr = frame->CopyConvertedFrameDataToArray(imgSize, reinterpret_cast<BYTE*>(colorImage.data), ColorImageFormat_Bgra);
					}
				}
				SafeRelease(frameDesc);
				SafeRelease(frame);
			}
		}
		return colorImage;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////
	cv::Mat CKinectStreamsMat::getDepthFrame(IDepthFrameReader * _depth_reader, HRESULT sourceInitHresult, bool rawData)
	{
		HRESULT hr = sourceInitHresult;
		IDepthFrame* frame = nullptr;
		cv::Mat depthImage;
		IFrameDescription* frame_desc = nullptr;
		if (SUCCEEDED(hr)) {
			hr = _depth_reader->AcquireLatestFrame(&frame);
			if (SUCCEEDED(hr)) {
				hr = frame->get_FrameDescription(&frame_desc);
				if (SUCCEEDED(hr)) {
					int frameWidth = 0, frameHeight = 0;
					hr = frame_desc->get_Width(&frameWidth);
					if (SUCCEEDED(hr))
					{
						hr = frame_desc->get_Height(&frameHeight);
					}
					if (SUCCEEDED(hr)) {
						const int imgSize = frameWidth*frameHeight;
						UINT16* pixelData = new UINT16[imgSize];
						hr = frame->CopyFrameDataToArray(imgSize, pixelData);
						if (SUCCEEDED(hr) && !rawData) {
							depthImage = cv::Mat(frameHeight, frameWidth, CV_8U);
							for (int i = 0; i < imgSize; i++) {
								UINT16 depth = pixelData[i];
								depthImage.at<UINT8>(i) = depth & 0xffff;
							}
						}
						else if (SUCCEEDED(hr) && rawData) {
							depthImage = cv::Mat(frameHeight, frameWidth, CV_16U);
							for (int i = 0; i < imgSize; i++) {
								depthImage.at<UINT16>(i) = pixelData[i];
							}
						}

						delete[] pixelData;
					}
				}
				SafeRelease(frame_desc);
				SafeRelease(frame);
			}
		}
		return depthImage;
	}


	////////////////////////////////////////////////////////////////////////////////////////////////
	cv::Mat CKinectStreamsMat::getIRframe(IInfraredFrameReader * _ir_reader, HRESULT sourceInitHresult)
	{
		HRESULT hr = sourceInitHresult;
		IInfraredFrame* frame = nullptr;
		IFrameDescription* frameDesc;
		cv::Mat infraredImage;
		if (SUCCEEDED(hr)) {
			hr = _ir_reader->AcquireLatestFrame(&frame);
			if (SUCCEEDED(hr)) {
				unsigned short* frameData = nullptr;
				hr = frame->get_FrameDescription(&frameDesc);
				if (SUCCEEDED(hr)) {
					int frameWidth = 0, frameHeight = 0;
					hr = frameDesc->get_Width(&frameWidth);
					if (SUCCEEDED(hr)) {
						hr = frameDesc->get_Height(&frameHeight);
					}
					if (SUCCEEDED(hr)) {
						UINT imgSize = 0;
						infraredImage = cv::Mat(frameHeight, frameWidth, CV_8UC1);
						hr = frame->AccessUnderlyingBuffer(&imgSize, &frameData);
					}
					if (SUCCEEDED(hr)) {
						int y = 0;
						int x = 0;
						FORI(y,frameHeight) {
							FORI(x,frameWidth) {
								unsigned int index = y * frameWidth + x;
								infraredImage.at<unsigned char>(y, x) = frameData[index] >> 6;
							}
						}
					}
				}
				SafeRelease(frameDesc);
			}
			SafeRelease(frame);
		}
		return infraredImage;
	}

}
