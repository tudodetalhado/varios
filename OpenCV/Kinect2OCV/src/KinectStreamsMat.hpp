#pragma once

#include "helper.hpp"

namespace K2OCV {

	class CKinectStreamsMat {
	
	public:
		/********************     Functions   *************************/

	//Obtain Color Frame from Kinect If Color Source Successfuly Initialized
	cv::Mat getColorFrame(IColorFrameReader * _color_reader, HRESULT sourceInitHresult);

	//Obtain Depth Frame from Kinect If Depth Source Successfuly Initialized
	cv::Mat getDepthFrame(IDepthFrameReader * _depth_reader, HRESULT sourceInitHresult, bool rawData = false);

	//Obtain Infrared Frame from Kinect If IR Source Successfuly Initialized
	cv::Mat getIRframe(IInfraredFrameReader * _ir_reader, HRESULT sourceInitHresult);

	};


}
