#pragma once

#include "helper.hpp"

namespace K2OCV {

	class CKinectFaceMat {
		//Array to Hold Face Coordinates
	cv::Rect faceRects[BODY_COUNT];
	public:

		//Obtain High Definition Face Frame from Kinect
	cv::Rect* getHDfaceRect();			/********    To be implemented later   *********/

		//Obtain Standard Face Frame from Kinect
	cv::Rect* getSDFaceRect(IBodyFrameReader* _body_reader, IFaceFrameReader* _face_reader[],
		IFaceFrameSource* _face_source[], int& trackedFaces, HRESULT faceReaderInit, HRESULT bodyReaderInit, bool color);
	};


}
