// ControlBasicCPPFinal.cpp : Defines the entry point for the console application.
//from internet   http://social.msdn.microsoft.com/Forums/en-US/kinectsdknuiapi/thread/e4f5a696-ed4f-4a5f-8e54-4b3706f62ad0
//DsoTsen
#include "stdafx.h"
#include <conio.h>
#include <Windows.h>

#include <iostream>
#include <NuiApi.h>
#include <KinectInteraction.h>
using namespace std;
#define SafeRelease(X) if(X) delete X;
//----------------------------------------------------
//#define _WINDOWS
INuiSensor            *m_pNuiSensor;

INuiInteractionStream *m_nuiIStream;
class CIneractionClient:public INuiInteractionClient
{
public:
	CIneractionClient()
	{;}
	~CIneractionClient()
	{;}
	STDMETHOD(GetInteractionInfoAtLocation)(THIS_ DWORD skeletonTrackingId, NUI_HAND_TYPE handType, FLOAT x, FLOAT y, _Out_ NUI_INTERACTION_INFO *pInteractionInfo)
	{        
		if(pInteractionInfo)
		{
			pInteractionInfo->IsPressTarget         = TRUE;//must add
			pInteractionInfo->IsGripTarget          = FALSE;    //must add
// 			pInteractionInfo->PressTargetControlId  = 0;
// 			pInteractionInfo->PressAttractionPointX = 0.f;
// 			pInteractionInfo->PressAttractionPointY = 0.f;
			return S_OK;
		}
		return E_POINTER;
		//return S_OK; 
	}

	STDMETHODIMP_(ULONG)    AddRef()                                    { return 2;     }
	STDMETHODIMP_(ULONG)    Release()                                   { return 1;     }
	STDMETHODIMP            QueryInterface(REFIID riid, void **ppv)     { return S_OK;  }

};

CIneractionClient m_nuiIClient;
//--------------------------------------------------------------------
HANDLE m_hNextColorFrameEvent;
HANDLE m_hNextDepthFrameEvent;
HANDLE m_hNextSkeletonEvent;
HANDLE m_hNextInteractionEvent;
HANDLE m_pColorStreamHandle;
HANDLE m_pDepthStreamHandle;
HANDLE m_hEvNuiProcessStop;
//-----------------------------------------------------------------------------------

int DrawColor(HANDLE h)
{
	return 0;
}

int DrawDepth(HANDLE h)
{
	NUI_IMAGE_FRAME pImageFrame;
	INuiFrameTexture* pDepthImagePixelFrame;
	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame( h, 0, &pImageFrame );
	BOOL nearMode = TRUE;
	m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_pDepthStreamHandle, &pImageFrame, &nearMode, &pDepthImagePixelFrame);
	INuiFrameTexture * pTexture = pDepthImagePixelFrame;
	NUI_LOCKED_RECT LockedRect;  
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );  
	if( LockedRect.Pitch != 0 )
	{
		HRESULT hr = m_nuiIStream->ProcessDepth(LockedRect.size,PBYTE(LockedRect.pBits),pImageFrame.liTimeStamp);
		if( FAILED( hr ) )
		{
			cout<<"Process Depth failed"<<endl;
		}
	}
	pTexture->UnlockRect(0);
	m_pNuiSensor->NuiImageStreamReleaseFrame( h, &pImageFrame );
	return 0;
}

int DrawSkeleton()
{
	NUI_SKELETON_FRAME SkeletonFrame = {0};
	HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame( 0, &SkeletonFrame );
	if( FAILED( hr ) )
	{
		cout<<"Get Skeleton Image Frame Failed"<<endl;
		return -1;
	}
	bool bFoundSkeleton = true;
	static int static_one_is_enough=0;
	if(static_one_is_enough==0)
	{
		cout<<"find skeleton !"<<endl;
		static_one_is_enough++;
	}
	m_pNuiSensor->NuiTransformSmooth(&SkeletonFrame,NULL); 
	Vector4 vTemp;
	m_pNuiSensor->NuiAccelerometerGetCurrentReading(&vTemp);
	hr =m_nuiIStream->ProcessSkeleton(NUI_SKELETON_COUNT, 
		SkeletonFrame.SkeletonData,
		&vTemp,
		SkeletonFrame.liTimeStamp);
	if( FAILED( hr ) )
	{
		cout<<"Process Skeleton failed"<<endl;
	}

	return 0;
}

int ShowInteraction()
{
	NUI_INTERACTION_FRAME Interaction_Frame;
	HRESULT hr = m_nuiIStream->GetNextFrame( 0,&Interaction_Frame );
	if(hr != S_OK)
	{
		if(hr == E_POINTER)
			cout<<"E_POINTER          "<<endl;
		else if(hr == E_NUI_FRAME_NO_DATA)
		{
			cout<<"E_NUI_FRAME_NO_DATA"<<endl;
		}
		return -1;
	}
	int trackingID = 0;
	int event;
	for(int i=0 ; i<NUI_SKELETON_COUNT ; i++)
	{
		COORD pos = {0,0};
		HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);    //º¯Êý¾ä±ú
		SetConsoleCursorPosition(hOut, pos);   
		static int frameCount=0;
		frameCount++;
		for(int j=0;j<NUI_USER_HANDPOINTER_COUNT;j++)
		{
			if( ( frameCount%3 )==1 )
			{
				trackingID = Interaction_Frame.UserInfos[i].SkeletonTrackingId;
				event = Interaction_Frame.UserInfos[i].HandPointerInfos[j].HandEventType;
				DWORD state = Interaction_Frame.UserInfos[i].HandPointerInfos[j].State;
				NUI_HAND_TYPE type = Interaction_Frame.UserInfos[i].HandPointerInfos[j].HandType;
				if(type==NUI_HAND_TYPE_NONE)
					continue;
				if((state&&NUI_HANDPOINTER_STATE_TRACKED)==0)
					continue;
				if((state&&NUI_HANDPOINTER_STATE_ACTIVE)==0)
					continue;
				cout<<"id="<<trackingID<<"--------HandEventType=";
				if(event == NUI_HAND_EVENT_TYPE_GRIP)
				{
					cout<<"Grip £¡£¡£¡   ";
				}
				else if(event == NUI_HAND_EVENT_TYPE_GRIPRELEASE)
				{
					cout<<"Grip Release "; 
				}
				else
				{
					cout<<"No Event!    ";
				}
				cout<<"    HandType=";

				if(type==NUI_HAND_TYPE_NONE)
					cout<<"No    Hand";
				else if(type==NUI_HAND_TYPE_LEFT)
					cout<<"Left  Hand";
				else if(type==NUI_HAND_TYPE_RIGHT)
					cout<<"Right Hand";
				cout<<endl;
				//////NUI_HANDPOINTER_STATE
				cout<<"STATE_TRACKED =      ";
				if((state&&NUI_HANDPOINTER_STATE_TRACKED)==1)
					cout<<"   TRACKED!";
				else
					cout<<" No TRACKED";
				cout<<endl;
				cout<<"STATE_ACTIVE =       ";
				if((state&&NUI_HANDPOINTER_STATE_ACTIVE)==1)
					cout<<"     ACTIVE";
				else
					cout<<" Not ACTIVE";
				cout<<endl;
				cout<<"STATE_INTERACTIVE =  ";
				if((state&&NUI_HANDPOINTER_STATE_INTERACTIVE)==1)
					cout<<"    INTERACTIVE!";
				else
					cout<<" Not INTERACTIVE";
				cout<<endl;
				cout<<"STATE_PRESSED =      ";
				if((state&&NUI_HANDPOINTER_STATE_PRESSED)==1)
					cout<<"    PRESSED!";
				else
					cout<<" Not PRESSED";
				cout<<endl;
				cout<<"PRIMARY_FOR_USER =   ";
				if((state&&NUI_HANDPOINTER_STATE_PRIMARY_FOR_USER)==1)
					cout<<"    PRIMARY!";
				else
					cout<<" Not PRIMARY";
				cout<<endl;
				//	system("\f");
			}
		}

	}

	return 0;
}

DWORD WINAPI KinectDataThread(LPVOID pParam)
{
	HANDLE hEvents[5] = {m_hEvNuiProcessStop,m_hNextColorFrameEvent,
		m_hNextDepthFrameEvent,m_hNextSkeletonEvent,m_hNextInteractionEvent};

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
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0))
		{
			DrawColor(m_pColorStreamHandle);
		}
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0))
		{
			DrawDepth(m_pDepthStreamHandle);
		}
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextSkeletonEvent, 0))
		{
			DrawSkeleton();
		}
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextInteractionEvent, 0))
		{
			ShowInteraction();
		}
	}

	CloseHandle(m_hEvNuiProcessStop);
	m_hEvNuiProcessStop = NULL;
	CloseHandle( m_hNextSkeletonEvent );
	CloseHandle( m_hNextDepthFrameEvent );
	CloseHandle( m_hNextColorFrameEvent );
	CloseHandle( m_hNextInteractionEvent );
	return 0;
}

DWORD ConnectKinect()
{
	INuiSensor * pNuiSensor;
	HRESULT hr;
	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);
	if (FAILED(hr))
	{
		return hr;
	}
	// Look at each Kinect sensor
	for (int i = 0; i < iSensorCount; ++i)
	{
		// Create the sensor so we can check status, if we can't create it, move on to the next
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}
		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)
		{
			m_pNuiSensor = pNuiSensor;
			break;
		}
		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}
	if (NULL != m_pNuiSensor)
	{
		if (SUCCEEDED(hr))
		{   
			hr = m_pNuiSensor->NuiInitialize(\
				NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX|\
				NUI_INITIALIZE_FLAG_USES_COLOR|\
				NUI_INITIALIZE_FLAG_USES_SKELETON);
			if( hr != S_OK )
			{
				cout<<"NuiInitialize failed"<<endl;
				return hr;
			}

			m_hNextColorFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
			m_pColorStreamHandle = NULL;

			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_COLOR,NUI_IMAGE_RESOLUTION_640x480, 0, 2, 
				m_hNextColorFrameEvent, &m_pColorStreamHandle);
			if( FAILED( hr ) )
			{
				cout<<"Could not open image stream video"<<endl;
				return hr;
			}

			m_hNextDepthFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
			m_pDepthStreamHandle = NULL;

			hr = m_pNuiSensor->NuiImageStreamOpen( 
				NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
				NUI_IMAGE_RESOLUTION_640x480, 0, 2, 
				m_hNextDepthFrameEvent, &m_pDepthStreamHandle);
			if( FAILED( hr ) )
			{
				cout<<"Could not open depth stream video"<<endl;
				return hr;
			}
			m_hNextSkeletonEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
			hr = m_pNuiSensor->NuiSkeletonTrackingEnable( 
				m_hNextSkeletonEvent, 
				NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE//|
				);
			if( FAILED( hr ) )
			{
				cout<<"Could not open skeleton stream video"<<endl;
				return hr;
			}
		}
	}
	if (NULL == m_pNuiSensor || FAILED(hr))
	{
		cout<<"No ready Kinect found!"<<endl;
		return E_FAIL;
	}
	return hr;
}

int main()
{
	ConnectKinect();
	HRESULT hr;
	m_hNextInteractionEvent = CreateEvent( NULL,TRUE,FALSE,NULL );
	m_hEvNuiProcessStop = CreateEvent(NULL,TRUE,FALSE,NULL);
	hr = NuiCreateInteractionStream(m_pNuiSensor,(INuiInteractionClient *)&m_nuiIClient,&m_nuiIStream);
	if( FAILED( hr ) )
	{
		cout<<"Could not open Interation stream video"<<endl;
		return hr;
	}
	hr = m_nuiIStream->Enable(m_hNextInteractionEvent);////////????
	if( FAILED( hr ) )
	{
		cout<<"Could not open Interation stream video"<<endl;
		return hr;
	}
	HANDLE m_hProcesss = CreateThread(NULL, 0, KinectDataThread, 0, 0, 0);
	while(1)
	{
		Sleep(1);
	}
	m_pNuiSensor->NuiShutdown();
	SafeRelease(m_pNuiSensor);
	return 0;
}