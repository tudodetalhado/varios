#include "KinectFaceMat.hpp"

namespace K2OCV 

{
	/////TO BE IMPLEMENTED LATER///////////
	cv::Rect * CKinectFaceMat::getHDfaceRect() { return nullptr; } 

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	cv::Rect* CKinectFaceMat::getSDFaceRect(IBodyFrameReader* _body_reader, IFaceFrameReader* _face_reader[],
		IFaceFrameSource* _face_source[], int& trackedFaces, HRESULT faceReaderInit, HRESULT bodyReaderInit, bool color) 
	{
		HRESULT hResult;
		if (SUCCEEDED(faceReaderInit) && SUCCEEDED(bodyReaderInit)) {
			IBodyFrame* pBodyFrame = nullptr;
			hResult = _body_reader->AcquireLatestFrame(&pBodyFrame);
			if (SUCCEEDED(hResult)) {
				IBody* pBody[BODY_COUNT] = { 0 };
				hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
				if (SUCCEEDED(hResult)) {

					FOR02CNT(BODY_COUNT) {
						BOOLEAN bTracked = false;
						hResult = pBody[count]->get_IsTracked(&bTracked);
						if (SUCCEEDED(hResult) && bTracked) {
						
							// Set TrackingID to Detect Face
							UINT64 trackingId = _UI64_MAX;
							hResult = pBody[count]->get_TrackingId(&trackingId);
							if (SUCCEEDED(hResult)) {
								_face_source[count]->put_TrackingId(trackingId);
							}
						}
					}
				}
				FOR02CNT(BODY_COUNT) {
					SafeRelease(pBody[count]);
				}
			}
			SafeRelease(pBodyFrame);
			int iFace;
			FORI(iFace,BODY_COUNT){
				IFaceFrame* pFaceFrame = nullptr;
				hResult = _face_reader[iFace]->AcquireLatestFrame(&pFaceFrame);
				if (SUCCEEDED(hResult) && pFaceFrame != nullptr) {
					BOOLEAN bFaceTracked = false;
					hResult = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
					if (SUCCEEDED(hResult) && bFaceTracked) {
						IFaceFrameResult* pFaceResult = nullptr;
						hResult = pFaceFrame->get_FaceFrameResult(&pFaceResult);
						if (SUCCEEDED(hResult) && pFaceResult != nullptr) {
							RectI faceBox;
							if (color) {
								hResult = pFaceResult->get_FaceBoundingBoxInColorSpace(&faceBox);
							}
							else {
								hResult = pFaceResult->get_FaceBoundingBoxInInfraredSpace(&faceBox);
							}
							if (SUCCEEDED(hResult)) {
								trackedFaces++;
								faceRects[iFace].x = faceBox.Left;
								faceRects[iFace].y = faceBox.Top;
								faceRects[iFace].width = faceBox.Right - faceBox.Left;
								faceRects[iFace].height = faceBox.Bottom - faceBox.Top;
							}
						}
						SafeRelease(pFaceResult);
					}
				}
				SafeRelease(pFaceFrame);
			}
		}
		return faceRects;
	}

}
