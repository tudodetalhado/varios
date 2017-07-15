// win32_KinectFaceTracking.cpp : �������̨Ӧ�ó������ڵ㡣
/****************************************************
������;��Kinect Face Tracking������
����������Visual Studio 2010		win32����
OpenCV2.4.4				��ʾ�����
Kinect SDK v1.6			�����汾
Windows 7				����ϵͳ
������Ա������
����ʱ�䣺2013-3-11 ~ 2013-3-12
��ϵ��ʽ��weibo.com/guoming0000
guoming0000@sina.com
www.ilovecode.cn
��ע������������ز������£�
Kinect Face Tracking SDK[Kinect��������]
******************************************************/
#include "stdafx.h"
#include <windows.h>
#include <opencv2\opencv.hpp>
#include <mmsystem.h>
#include <assert.h>
//#include <strsafe.h>
#include "NuiApi.h"
using namespace cv;
using namespace	std;
//----------------------------------------------------
#define  _WINDOWS
#include <FaceTrackLib.h>

//��ʾ��״����
HRESULT VisualizeFaceModel(IFTImage* pColorImg, IFTModel* pModel, FT_CAMERA_CONFIG const* pCameraConfig, FLOAT const* pSUCoef, 
	FLOAT zoomFactor, POINT viewOffset, IFTResult* pAAMRlt, UINT32 color);
//---ͼ���С�Ȳ���--------------------------------------------
#define COLOR_WIDTH		640
#define COLOR_HIGHT		480
#define DEPTH_WIDTH		320
#define DEPTH_HIGHT		240
#define SKELETON_WIDTH	640
#define SKELETON_HIGHT	480
#define CHANNEL			3
BYTE DepthBuf[DEPTH_WIDTH*DEPTH_HIGHT*CHANNEL];
//---face tracking����------------------------------------------
IFTImage*	pColorFrame,*pColorDisplay;		//��ɫͼ������
IFTImage*	pDepthFrame;					//���ͼ������
FT_VECTOR3D m_hint3D[2];					//ͷ�ͼ�����ĵ�����
//----�����ں��¼��;��-----------------------------------------------------------------
HANDLE	m_hNextVideoFrameEvent;
HANDLE	m_hNextDepthFrameEvent;
HANDLE	m_hNextSkeletonEvent;
HANDLE	m_pVideoStreamHandle;
HANDLE	m_pDepthStreamHandle;
HANDLE	m_hEvNuiProcessStop;//���ڽ������¼�����
//-----------------------------------------------------------------------------------
//��ȡ��ɫͼ�����ݣ���������ʾ
int DrawColor(HANDLE h)
{
	const NUI_IMAGE_FRAME * pImageFrame = NULL;
	HRESULT hr = NuiImageStreamGetNextFrame( h, 0, &pImageFrame );
	if( FAILED( hr ) )
	{
		cout<<"Get Color Image Frame Failed"<<endl;
		return -1;
	}
	INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );
	if( LockedRect.Pitch != 0 )
	{
		BYTE * pBuffer = (BYTE*) LockedRect.pBits;
		memcpy(pColorFrame->GetBuffer(), PBYTE(LockedRect.pBits), 
			min(pColorFrame->GetBufferSize(), UINT(pTexture->BufferLen())));
		//OpenCV��ʾ��ɫ��Ƶ
		Mat temp(COLOR_HIGHT,COLOR_WIDTH,CV_8UC4,pBuffer);
		imshow("ColorVideo",temp);
		int c = waitKey(1);//����ESC����
		//�������Ƶ���水��ESC,q,Q���ᵼ�����������˳�
		if( c == 27 || c == 'q' || c == 'Q' )
		{
			SetEvent(m_hEvNuiProcessStop);
		}
	}
	NuiImageStreamReleaseFrame( h, pImageFrame );
	return 0;
}
//��ȡ���ͼ�����ݣ���������ʾ
int DrawDepth(HANDLE h)
{
	const NUI_IMAGE_FRAME * pImageFrame = NULL;
	HRESULT hr = NuiImageStreamGetNextFrame( h, 0, &pImageFrame );
	if( FAILED( hr ) )
	{
		cout<<"Get Depth Image Frame Failed"<<endl;
		return -1;
	}
	INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );
	if( LockedRect.Pitch != 0 )
	{
		USHORT * pBuff = (USHORT*) LockedRect.pBits;
		//		pDepthBuffer = pBuff;
		memcpy(pDepthFrame->GetBuffer(), PBYTE(LockedRect.pBits), 
			min(pDepthFrame->GetBufferSize(), UINT(pTexture->BufferLen())));

		for(int i=0;i<DEPTH_WIDTH*DEPTH_HIGHT;i++)
		{
			BYTE index = pBuff[i]&0x07;
			USHORT realDepth = (pBuff[i]&0xFFF8)>>3;
			BYTE scale = 255 - (BYTE)(256*realDepth/0x0fff);
			DepthBuf[CHANNEL*i] = DepthBuf[CHANNEL*i+1] = DepthBuf[CHANNEL*i+2] = 0;
			switch( index )
			{
			case 0:
				DepthBuf[CHANNEL*i]=scale/2;
				DepthBuf[CHANNEL*i+1]=scale/2;
				DepthBuf[CHANNEL*i+2]=scale/2;
				break;
			case 1:
				DepthBuf[CHANNEL*i]=scale;
				break;
			case 2:
				DepthBuf[CHANNEL*i+1]=scale;
				break;
			case 3:
				DepthBuf[CHANNEL*i+2]=scale;
				break;
			case 4:
				DepthBuf[CHANNEL*i]=scale;
				DepthBuf[CHANNEL*i+1]=scale;
				break;
			case 5:
				DepthBuf[CHANNEL*i]=scale;
				DepthBuf[CHANNEL*i+2]=scale;
				break;
			case 6:
				DepthBuf[CHANNEL*i+1]=scale;
				DepthBuf[CHANNEL*i+2]=scale;
				break;
			case 7:
				DepthBuf[CHANNEL*i]=255-scale/2;
				DepthBuf[CHANNEL*i+1]=255-scale/2;
				DepthBuf[CHANNEL*i+2]=255-scale/2;
				break;
			}
		}
		Mat temp(DEPTH_HIGHT,DEPTH_WIDTH,CV_8UC3,DepthBuf);
		imshow("DepthVideo",temp);
		int c = waitKey(1);//����ESC����
		if( c == 27 || c == 'q' || c == 'Q' )
		{
			SetEvent(m_hEvNuiProcessStop);
		}
	}
	NuiImageStreamReleaseFrame( h, pImageFrame );
	return 0;
}
//��ȡ�������ݣ���������ʾ
int DrawSkeleton()
{
	NUI_SKELETON_FRAME SkeletonFrame;
	cv::Point pt[20];
	Mat skeletonMat=Mat(SKELETON_HIGHT,SKELETON_WIDTH,CV_8UC3,Scalar(0,0,0));
	HRESULT hr = NuiSkeletonGetNextFrame( 0, &SkeletonFrame );
	if( FAILED( hr ) )
	{
		cout<<"Get Skeleton Image Frame Failed"<<endl;
		return -1;
	}
	bool bFoundSkeleton = false;
	for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
	{
		if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
		{
			bFoundSkeleton = true;
		}
	}
	// Has skeletons!
	if( bFoundSkeleton )
	{
		NuiTransformSmooth(&SkeletonFrame,NULL);
		for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
		{
			if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
			{
				for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++)
				{
					float fx,fy;
					NuiTransformSkeletonToDepthImage( SkeletonFrame.SkeletonData[i].SkeletonPositions[j], &fx, &fy );
					pt[j].x = (int) ( fx * SKELETON_WIDTH )/320;
					pt[j].y = (int) ( fy * SKELETON_HIGHT )/240;
					circle(skeletonMat,pt[j],5,CV_RGB(255,0,0));
				}
				cout<<"one people"<<endl;
		//		 cv::line(skeletonMat,pt[NUI_SKELETON_POSITION_SHOULDER_CENTER],pt[NUI_SKELETON_POSITION_SPINE],CV_RGB(0,255,0));
		//		cv::line(skeletonMat,pt[NUI_SKELETON_POSITION_SPINE],pt[NUI_SKELETON_POSITION_HIP_CENTER],CV_RGB(0,255,0));  

				cv::line(skeletonMat,pt[NUI_SKELETON_POSITION_HEAD],pt[NUI_SKELETON_POSITION_SHOULDER_CENTER],CV_RGB(0,255,0));
				cv::line(skeletonMat,pt[NUI_SKELETON_POSITION_HAND_RIGHT],pt[NUI_SKELETON_POSITION_WRIST_RIGHT],CV_RGB(0,255,0));
				cv::line(skeletonMat,pt[NUI_SKELETON_POSITION_WRIST_RIGHT],pt[NUI_SKELETON_POSITION_ELBOW_RIGHT],CV_RGB(0,255,0));
				cv::line(skeletonMat,pt[NUI_SKELETON_POSITION_ELBOW_RIGHT],pt[NUI_SKELETON_POSITION_SHOULDER_RIGHT],CV_RGB(0,255,0));
				cv::line(skeletonMat,pt[NUI_SKELETON_POSITION_SHOULDER_RIGHT],pt[NUI_SKELETON_POSITION_SHOULDER_CENTER],CV_RGB(0,255,0));
				cv::line(skeletonMat,pt[NUI_SKELETON_POSITION_SHOULDER_CENTER],pt[NUI_SKELETON_POSITION_SHOULDER_LEFT],CV_RGB(0,255,0));
				cv::line(skeletonMat,pt[NUI_SKELETON_POSITION_SHOULDER_LEFT],pt[NUI_SKELETON_POSITION_ELBOW_LEFT],CV_RGB(0,255,0));
				cv::line(skeletonMat,pt[NUI_SKELETON_POSITION_ELBOW_LEFT],pt[NUI_SKELETON_POSITION_WRIST_LEFT],CV_RGB(0,255,0));
				cv::line(skeletonMat,pt[NUI_SKELETON_POSITION_WRIST_LEFT],pt[NUI_SKELETON_POSITION_HAND_LEFT],CV_RGB(0,255,0));
				m_hint3D[0].x=SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].x;
				m_hint3D[0].y=SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].y;
				m_hint3D[0].z=SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].z;
				m_hint3D[1].x=SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD].x;
				m_hint3D[1].y=SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD].y;
				m_hint3D[1].z=SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD].z;
			}
		}
	}
	imshow("SkeletonVideo",skeletonMat);
	waitKey(1);
	int c = waitKey(1);//����ESC����
	if( c == 27 || c == 'q' || c == 'Q' )
	{
		SetEvent(m_hEvNuiProcessStop);
	}
	return 0;
}

DWORD WINAPI KinectDataThread(LPVOID pParam)
{
	HANDLE hEvents[4] = {m_hEvNuiProcessStop,m_hNextVideoFrameEvent,
		m_hNextDepthFrameEvent,m_hNextSkeletonEvent};
	while(1)
	{
		int nEventIdx;
		nEventIdx=WaitForMultipleObjects(sizeof(hEvents)/sizeof(hEvents[0]),
										hEvents,FALSE,100);
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hEvNuiProcessStop, 0))
		{
			break;
		}
		// Process signal events
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextVideoFrameEvent, 0))
		{
			DrawColor(m_pVideoStreamHandle);
		}
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0))
		{
			DrawDepth(m_pDepthStreamHandle);
		}
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextSkeletonEvent, 0))
		{
			DrawSkeleton();
		}
//���ַ�ʽ�رճ���ʱ���ܳ�����
// 		switch(nEventIdx)
// 		{
// 		case 0:
// 			break;
// 		case 1:
// 			DrawColor(m_pVideoStreamHandle);
// 		case 2:
// 			DrawDepth(m_pDepthStreamHandle);
// 		case 3:
// 			DrawSkeleton();
// 		}
	}
	CloseHandle(m_hEvNuiProcessStop);
	m_hEvNuiProcessStop = NULL;
	CloseHandle( m_hNextSkeletonEvent );
	CloseHandle( m_hNextDepthFrameEvent );
	CloseHandle( m_hNextVideoFrameEvent );
	return 0;
}
int main(int argc,char * argv[])
{
	m_hint3D[0].x=0;
	m_hint3D[0].y=0;
	m_hint3D[0].z=0;
	m_hint3D[1].x=0;
	m_hint3D[1].y=0;
	m_hint3D[1].z=0;
	//��ʼ��NUI
	HRESULT hr = NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX|NUI_INITIALIZE_FLAG_USES_COLOR|NUI_INITIALIZE_FLAG_USES_SKELETON);
	if( hr != S_OK )
	{
		cout<<"NuiInitialize failed"<<endl;
		return hr;
	}
	//��KINECT�豸�Ĳ�ɫͼ��Ϣͨ��
	m_hNextVideoFrameEvent	= CreateEvent( NULL, TRUE, FALSE, NULL );
	m_pVideoStreamHandle	= NULL;
	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,NUI_IMAGE_RESOLUTION_640x480, 0, 2, m_hNextVideoFrameEvent, &m_pVideoStreamHandle);
	if( FAILED( hr ) )
	{
		cout<<"Could not open image stream video"<<endl;
		return hr;
	}
	m_hNextDepthFrameEvent	= CreateEvent( NULL, TRUE, FALSE, NULL );
	m_pDepthStreamHandle	= NULL;
	hr = NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, 0, 2, m_hNextDepthFrameEvent, &m_pDepthStreamHandle);
	if( FAILED( hr ) )
	{
		cout<<"Could not open depth stream video"<<endl;
		return hr;
	}
	m_hNextSkeletonEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	hr = NuiSkeletonTrackingEnable( m_hNextSkeletonEvent, 
		NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE|NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);
	if( FAILED( hr ) )
	{
		cout<<"Could not open skeleton stream video"<<endl;
		return hr;
	}
	m_hEvNuiProcessStop = CreateEvent(NULL,TRUE,FALSE,NULL);//���ڽ������¼�����
	//����һ���߳�---���ڶ�ȡ��ɫ����ȡ���������
	HANDLE m_hProcesss = CreateThread(NULL, 0, KinectDataThread, 0, 0, 0);
	///////////////////////////////////////////////
	m_hint3D[0] = FT_VECTOR3D(0, 0, 0);
	m_hint3D[1] = FT_VECTOR3D(0, 0, 0);

	pColorFrame		= FTCreateImage();
	pDepthFrame		= FTCreateImage();
	pColorDisplay	= FTCreateImage();
	IFTFaceTracker* pFT = FTCreateFaceTracker();
	if(!pFT)
	{
		return -1;// Handle errors
	}
	//��ʼ��������������Ҫ����������
	FT_CAMERA_CONFIG myCameraConfig = {COLOR_WIDTH, COLOR_HIGHT, NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS}; // width, height, focal length
	FT_CAMERA_CONFIG depthConfig;
	depthConfig.FocalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
	depthConfig.Width		= DEPTH_WIDTH;
	depthConfig.Height		= DEPTH_HIGHT;//����һ��Ҫ�����Ҫ��Բ��У���
	//IFTFaceTracker�ĳ�ʼ��
	hr = pFT->Initialize(&myCameraConfig, &depthConfig, NULL, NULL);
	if( FAILED(hr) )
	{
		return -2;// Handle errors
	}
	// Create IFTResult to hold a face tracking result
	IFTResult* pFTResult = NULL;
	hr = pFT->CreateFTResult(&pFTResult);
	if(FAILED(hr))
	{
		return -11;
	}
	// prepare Image and SensorData for 640x480 RGB images
	if(!pColorFrame)
	{
		return -12;// Handle errors
	}
	// Attach assumes that the camera code provided by the application
	// is filling the buffer cameraFrameBuffer
	//�����ڴ�ռ�
	pColorDisplay->Allocate(COLOR_WIDTH, COLOR_HIGHT, FTIMAGEFORMAT_UINT8_B8G8R8X8);
	hr = pColorFrame->Allocate(COLOR_WIDTH, COLOR_HIGHT, FTIMAGEFORMAT_UINT8_B8G8R8X8);
	if (FAILED(hr))
	{
		return hr;
	}
	hr = pDepthFrame->Allocate(DEPTH_WIDTH, DEPTH_HIGHT, FTIMAGEFORMAT_UINT16_D13P3);
	if (FAILED(hr))
	{
		return hr;
	}
	//���FT_SENSOR_DATA�ṹ
	FT_SENSOR_DATA			sensorData;
	POINT					point;
	sensorData.ZoomFactor	= 1.0f;
	point.x					= 0;
	point.y					= 0;
	sensorData.ViewOffset	= point;

	bool	isTracked		= false;
	int		iFaceTrackTimeCount=0;
	// ��������
	while ( true )
	{
		sensorData.pVideoFrame = pColorFrame;
		sensorData.pDepthFrame = pDepthFrame;
		if(!isTracked)
		{
			//��ķѽ϶�cpu������Դ
			hr = pFT->StartTracking(&sensorData, NULL, m_hint3D, pFTResult);
			if(SUCCEEDED(hr) && SUCCEEDED(pFTResult->GetStatus()))
			{
				isTracked = true;
			}
			else
			{
				isTracked = false;
			}
		}
		else
		{
			//����׷�٣���һ��ʹ��һ���Ѵ��֪��������ģ�ͣ��������ĵ��ò������Ķ���cpu����
			hr = pFT->ContinueTracking(&sensorData, m_hint3D, pFTResult);
			if(FAILED(hr) || FAILED (pFTResult->GetStatus()))
			{
				// Handle errors
				isTracked = false;
			}
		}
		if(isTracked)
		{
			IFTModel*	ftModel;
			HRESULT		hr = pFT->GetFaceModel(&ftModel);
			FLOAT*		pSU = NULL;
			UINT		numSU;
			BOOL		suConverged;
			pFT->GetShapeUnits(NULL, &pSU, &numSU, &suConverged);
			POINT viewOffset = {0, 0};
			pColorFrame->CopyTo(pColorDisplay,NULL,0,0);
			hr = VisualizeFaceModel(pColorDisplay, ftModel, &myCameraConfig, pSU, 1.0, viewOffset, pFTResult, 0x00FFFF00);
			if(FAILED(hr))
				printf("��ʾʧ�ܣ���\n");
			Mat tempMat(COLOR_HIGHT,COLOR_WIDTH,CV_8UC4,pColorDisplay->GetBuffer());
			imshow("faceTracking",tempMat);
			int c = waitKey(1);//����ESC����
			if(m_hEvNuiProcessStop!=NULL)
			{
				if( c == 27 || c == 'q' || c == 'Q' )
				{
					SetEvent(m_hEvNuiProcessStop);
					if(m_hProcesss!=NULL)
					{
						WaitForSingleObject(m_hProcesss,INFINITE);
						CloseHandle(m_hProcesss);
						m_hProcesss = NULL;
					}
					break;
				}
			}
			else
			{
				break;
			}
			//����ҲҪ�ж��Ƿ�m_hEvNuiProcessStop�Ѿ��������ˣ�
			//////
		}
		Sleep(16);
// 		iFaceTrackTimeCount++;
// 		if(iFaceTrackTimeCount>16*1000)
// 			break;
	}
	if(m_hProcesss!=NULL)
	{
		WaitForSingleObject(m_hProcesss,INFINITE);
		CloseHandle(m_hProcesss);
		m_hProcesss = NULL;
	}
	// Clean up.
	pFTResult->Release();
	pColorFrame->Release();
	pFT->Release();

	NuiShutdown();
	return 0;
}

HRESULT VisualizeFaceModel(IFTImage* pColorImg, IFTModel* pModel, FT_CAMERA_CONFIG const* pCameraConfig, FLOAT const* pSUCoef, 
	FLOAT zoomFactor, POINT viewOffset, IFTResult* pAAMRlt, UINT32 color)
{
	if (!pColorImg || !pModel || !pCameraConfig || !pSUCoef || !pAAMRlt)
	{
		return E_POINTER;
	}

	HRESULT hr = S_OK;
	UINT vertexCount = pModel->GetVertexCount();
	FT_VECTOR2D* pPts2D = reinterpret_cast<FT_VECTOR2D*>(_malloca(sizeof(FT_VECTOR2D) * vertexCount));
	if (pPts2D)
	{
		FLOAT *pAUs;
		UINT auCount;
		hr = pAAMRlt->GetAUCoefficients(&pAUs, &auCount);
		if (SUCCEEDED(hr))
		{
			//rotationXYZ������ת�Ƕȣ�
			FLOAT scale, rotationXYZ[3], translationXYZ[3];
			hr = pAAMRlt->Get3DPose(&scale, rotationXYZ, translationXYZ);
			if (SUCCEEDED(hr))
			{
				hr = pModel->GetProjectedShape(pCameraConfig, zoomFactor, viewOffset, pSUCoef, pModel->GetSUCount(), pAUs, auCount, 
					scale, rotationXYZ, translationXYZ, pPts2D, vertexCount);
				//�����ȡ��vertexCount���沿�����㣬�����pPts2Dָ��������
				if (SUCCEEDED(hr))
				{
					POINT* p3DMdl   = reinterpret_cast<POINT*>(_malloca(sizeof(POINT) * vertexCount));
					if (p3DMdl)
					{
						for (UINT i = 0; i < vertexCount; ++i)
						{
							p3DMdl[i].x = LONG(pPts2D[i].x + 0.5f);
							p3DMdl[i].y = LONG(pPts2D[i].y + 0.5f);
						}

						FT_TRIANGLE* pTriangles;
						UINT triangleCount;
						hr = pModel->GetTriangles(&pTriangles, &triangleCount);
						if (SUCCEEDED(hr))
						{
							struct EdgeHashTable
							{
								UINT32* pEdges;
								UINT edgesAlloc;

								void Insert(int a, int b) 
								{
									UINT32 v = (min(a, b) << 16) | max(a, b);
									UINT32 index = (v + (v << 8)) * 49157, i;
									for (i = 0; i < edgesAlloc - 1 && pEdges[(index + i) & (edgesAlloc - 1)] && v != pEdges[(index + i) & (edgesAlloc - 1)]; ++i)
									{
									}
									pEdges[(index + i) & (edgesAlloc - 1)] = v;
								}
							} eht;

							eht.edgesAlloc = 1 << UINT(log(2.f * (1 + vertexCount + triangleCount)) / log(2.f));
							eht.pEdges = reinterpret_cast<UINT32*>(_malloca(sizeof(UINT32) * eht.edgesAlloc));
							if (eht.pEdges)
							{
								ZeroMemory(eht.pEdges, sizeof(UINT32) * eht.edgesAlloc);
								for (UINT i = 0; i < triangleCount; ++i)
								{ 
									eht.Insert(pTriangles[i].i, pTriangles[i].j);
									eht.Insert(pTriangles[i].j, pTriangles[i].k);
									eht.Insert(pTriangles[i].k, pTriangles[i].i);
								}
								for (UINT i = 0; i < eht.edgesAlloc; ++i)
								{
									if(eht.pEdges[i] != 0)
									{
										pColorImg->DrawLine(p3DMdl[eht.pEdges[i] >> 16], p3DMdl[eht.pEdges[i] & 0xFFFF], color, 1);
									}
								}
								_freea(eht.pEdges);
							}

							// Render the face rect in magenta
							RECT rectFace;
							hr = pAAMRlt->GetFaceRect(&rectFace);
							if (SUCCEEDED(hr))
							{
								POINT leftTop = {rectFace.left, rectFace.top};
								POINT rightTop = {rectFace.right - 1, rectFace.top};
								POINT leftBottom = {rectFace.left, rectFace.bottom - 1};
								POINT rightBottom = {rectFace.right - 1, rectFace.bottom - 1};
								UINT32 nColor = 0xff00ff;
								SUCCEEDED(hr = pColorImg->DrawLine(leftTop, rightTop, nColor, 1)) &&
									SUCCEEDED(hr = pColorImg->DrawLine(rightTop, rightBottom, nColor, 1)) &&
									SUCCEEDED(hr = pColorImg->DrawLine(rightBottom, leftBottom, nColor, 1)) &&
									SUCCEEDED(hr = pColorImg->DrawLine(leftBottom, leftTop, nColor, 1));
							}
						}

						_freea(p3DMdl); 
					}
					else
					{
						hr = E_OUTOFMEMORY;
					}
				}
			}
		}
		_freea(pPts2D);
	}
	else
	{
		hr = E_OUTOFMEMORY;
	}
	return hr;
}
