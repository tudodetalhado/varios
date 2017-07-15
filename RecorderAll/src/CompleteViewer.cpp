//------------------------------------------------------------------------------
// <copyright file="CompleteViewer.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

//using OpenCV v 3.0 imgcodecs

#include <algorithm>
#include <Windows.h>
#include <tchar.h>
#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "CompleteViewer.h"
#include <ctime>
#include <iostream>
#include <stdio.h>
#include <cstdio>
#include <fstream>
#include <math.h>
#include <sstream>
#include <process.h>  
#include <locale>
#include <wchar.h>
#include <string>
#include <vector>
#include <comdef.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <thread>	
#include <chrono>
#include <cstring>		
#include <string.h>		
#include <stdlib.h>		
#include <cstdlib>
#include "windows.h"
#include <deque>
#include <array>
#include <memory>
#include <set>

using namespace std::this_thread; 
using namespace std::chrono;
using namespace cv;
using namespace std;

#define MAX_NUM_FRAME 45000			//Max number frames to capture and framerate
#define MAX_FPS 30					//Max framerate
#define RAM_PERC 0.8
#define SOFTWARELIMITSEC 900

char namepng[MAX_PATH];

//Frame dimensions
const int depth_row = 424;			//Row in depth frame
const int depth_col = 512;			//Column in depth frame
const int Color_row = 1080;			//Row in color frame
const int Color_col = 1920;			//Column in color frame
const int Mapp_cols = 512*2;		//Column in mapp table

//RGB Fps specs
LPTSTR NumFps;
int NumFpsCapture;
float recfps;

//Depth Fps specs          
LPTSTR NumFpsDepth;
int NumFpsCaptureDepth;
float recfpsDepth;

//IR Fps specs
LPTSTR NumFpsIR;
int NumFpsCaptureIR;
float recfpsIR;

//Skeleton Fps specs
LPTSTR NumFpsSkeleton;
int NumFpsCaptureSkeleton;
float recfpsSkeleton;

//Mapping Fps specs
LPTSTR NumFpsMapp;
int NumFpsCaptureMapp;
float recfpsMapp;

//Used for take number of frames to captures from gui
LPTSTR NumFramesFileName;
int NumFramesAllocate;

int NumFramesAllocateRGB;
int NumFramesAllocateIR;
int NumFramesAllocateDepth;
int NumFramesAllocateSkel;
int NumFramesAllocateMapp;

//Used for take number of capture time
LPTSTR NumAcquisitionTime;
int NumAcquisitionSec;

//Used for take path to save data from gui
LPTSTR PathFileName;

//Folder name
WCHAR FolderName[MAX_PATH];         //Root folder
WCHAR FolderNameTemp[MAX_PATH];     //Root temporary folder
WCHAR FolderName_m[MAX_PATH];       //Mapping data folder
WCHAR FolderName_c[MAX_PATH];       //Color data folder
WCHAR FolderName_d[MAX_PATH];       //Depth data folder
WCHAR FolderName_b[MAX_PATH];       //Body data folder
WCHAR FolderName_i[MAX_PATH];       //Infrared data folder
WCHAR FolderName_t[MAX_PATH];       //Timestamp folder
WCHAR FolderName_png[MAX_PATH];		//Png folder

//File name     
WCHAR TimeFileName[MAX_PATH];
WCHAR DataFileName[MAX_PATH];
WCHAR DataFileName2[MAX_PATH];
WCHAR DataFileName3[MAX_PATH];
WCHAR DataFileName4[MAX_PATH];
WCHAR FileNameSk[MAX_PATH];
WCHAR FileNameSkSpace[MAX_PATH];
WCHAR FileNameSkCsv[MAX_PATH];
WCHAR FileNameSkSpaceCsv[MAX_PATH];

//Stream for timestamp
ofstream FileColorTime;
ofstream FileDepthTime;
ofstream FileBodyTime;
ofstream FileInfraredTime;

//Stream for data
ofstream fileColor;			
ofstream filedepth;		
ofstream fileMapp;				
ofstream fileskeleton;												
ofstream fileskeletonSkSpace;
ofstream fileskeletonCsv;
ofstream fileskeletonSkSpaceCsv;

//Variables for skeleton
static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;
static const float c_InferredBoneThickness = 1.0f;
static const float c_HandSize = 30.0f;
int indexBody;
int temp = 0;
int frameBodyCounter = 0;

int64 secondsAvail ;
wchar_t bufferAvRam[5];

//Struct for body frame
typedef struct 
{
	Vector4 body[BODY_COUNT][JointType_Count];
	int jointState[BODY_COUNT][JointType_Count];
}   Body;

// Arrays for kinect timestamp
INT64 BodyTime[MAX_NUM_FRAME];
INT64 DepthTime[MAX_NUM_FRAME];
INT64 ColorTime[MAX_NUM_FRAME];
INT64 InfraredTime[MAX_NUM_FRAME];

// Arrays for qpc timestamp
LARGE_INTEGER clockTimestampBody[MAX_NUM_FRAME];
LARGE_INTEGER clockTimestampDepth[MAX_NUM_FRAME];
LARGE_INTEGER clockTimestampColor[MAX_NUM_FRAME];
LARGE_INTEGER clockTimestampIR[MAX_NUM_FRAME];

//Variables used for dynamic arrays
USHORT ** DepthArrayBuffer;
RGBQUAD ** ColorArrayBuffer;
USHORT ** MappingArrayBuffer;
RGBQUAD ** InfraredArrayBuffer;
Body * SkeletonArrayBuffer;
Body * SkeletonArrayBufferSkSpace;


//Buffer for body frame used in the body process when you are not capturing depth data
USHORT FrameAct[depth_row*depth_col];

//Flags used to choose the stream to capture
bool checkColor = 0;
bool checkIR = 0;
bool checkDepth = 0;
bool checkSkeleton = 0;
bool checkMapp = 0;	

//Flags used to start to capture data
bool CaptureIRFlag = 0;
bool CaptureColorFlag = 0;
bool CaptureSkeletonFlag = 0;
bool CaptureDepthFlag = 0;
bool CaptureMappFlag = 0;

//Counters frames captured
int TotalDepthFrameCaptured = 0;
int TotalColorFrameCaptured = 0;
int TotalSkeletonFrameCaptured = 0;
int TotalIRFrameCaptured = 0;
int TotalMappFrameCaptured = 0;

//Counters and flag used for gui
int contClickDepth = 0;
int contClickIR = 0;
int contClickSkeleton = 0;
int contClickMapp = 0;
int contClickBmp = 0;		
int contClickPng = 0;
bool ClickAllocates = 0;
bool ClickSave = 0;

//Declaration of thread
unsigned  __stdcall SaveThread(void * m_hWnd);
unsigned  __stdcall SaveThread2(void * m_hWnd);
unsigned  __stdcall SaveThread3(void * m_hWnd);
unsigned  __stdcall SaveThread4(void * m_hWnd);
unsigned IntervalThread1;
unsigned IntervalThread2;
unsigned IntervalThread3;
unsigned ThreadsSupported;
unsigned ThreadsMax = 1; //Choice option: 2 or 4 threads


void createAlphaMat(Mat &mat, int w)
{
	int val_pix;
    CV_Assert(mat.channels() == 4);
    for (int i = 0; i < mat.rows; ++i) 
    {
        for (int j = 0; j < mat.cols; ++j) 
        {
            Vec4b& bgra = mat.at<Vec4b>(i, j);            //Vec4b = vec uchar 4 channels
 
			val_pix = i*mat.cols + j;
            bgra[0] = ColorArrayBuffer[w][val_pix].rgbBlue; // Blue
            bgra[1] = ColorArrayBuffer[w][val_pix].rgbGreen; // Green
			bgra[2] = ColorArrayBuffer[w][val_pix].rgbRed; // Red
			bgra[3] = ColorArrayBuffer[w][val_pix].rgbReserved; // Alpha
        }
    }
}

void createAlphaMat2(Mat &mat2, int w)
{
int val_pix;
    CV_Assert(mat2.channels() == 4);
    for (int i = 0; i < mat2.rows; ++i) 
    {
        for (int j = 0; j < mat2.cols; ++j) 
        {
            Vec4b& bgra = mat2.at<Vec4b>(i, j);            //Vec4b = vec uchar 4 channels
 
			val_pix = i*mat2.cols + j;
            bgra[0] = ColorArrayBuffer[w][val_pix].rgbBlue; // Blue
            bgra[1] = ColorArrayBuffer[w][val_pix].rgbGreen; // Green
			bgra[2] = ColorArrayBuffer[w][val_pix].rgbRed; // Red
			bgra[3] = ColorArrayBuffer[w][val_pix].rgbReserved; // Alpha
        }
    }
}

void createAlphaMat3(Mat &mat3, int w)
{
int val_pix;
    CV_Assert(mat3.channels() == 4);
    for (int i = 0; i < mat3.rows; ++i) 
    {
        for (int j = 0; j < mat3.cols; ++j) 
        {
            Vec4b& bgra = mat3.at<Vec4b>(i, j);            //Vec4b = vec uchar 4 channels
 
			val_pix = i*mat3.cols + j;
            bgra[0] = ColorArrayBuffer[w][val_pix].rgbBlue; // Blue
            bgra[1] = ColorArrayBuffer[w][val_pix].rgbGreen; // Green
			bgra[2] = ColorArrayBuffer[w][val_pix].rgbRed; // Red
			bgra[3] = ColorArrayBuffer[w][val_pix].rgbReserved; // Alpha
        }
    }
}

void createAlphaMat4(Mat &mat4, int w)
{
int val_pix;
    CV_Assert(mat4.channels() == 4);
    for (int i = 0; i < mat4.rows; ++i) 
    {
        for (int j = 0; j < mat4.cols; ++j) 
        {
            Vec4b& bgra = mat4.at<Vec4b>(i, j);            //Vec4b = vec uchar 4 channels
 
			val_pix = i*mat4.cols + j;
            bgra[0] = ColorArrayBuffer[w][val_pix].rgbBlue; // Blue
            bgra[1] = ColorArrayBuffer[w][val_pix].rgbGreen; // Green
			bgra[2] = ColorArrayBuffer[w][val_pix].rgbRed; // Red
			bgra[3] = ColorArrayBuffer[w][val_pix].rgbReserved; // Alpha
        }
    }
}


/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    CCompleteViewer application;
    application.Run(hInstance, nCmdShow);
}

/// <summary>
/// Constructor
/// </summary>
CCompleteViewer::CCompleteViewer() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0),
	m_pMultiSourceFrameReader(NULL),
    m_pKinectSensor(NULL),
    m_pD2DFactory(NULL),
    m_pDrawDepth(NULL),
    m_pDepthRGBX(NULL),
	m_pColorRGBX(NULL),
    m_pRenderTarget(NULL),
    m_pBrushJointTracked(NULL),
    m_pBrushJointInferred(NULL),
    m_pBrushBoneTracked(NULL),
    m_pBrushBoneInferred(NULL),
    m_pBrushHandClosed(NULL),
    m_pBrushHandOpen(NULL),
    m_pBrushHandLasso(NULL),
    m_pDrawInfrared(NULL),
    m_pInfraredRGBX(NULL)
{

	MEMORYSTATUSEX memInfo;
    memInfo.dwLength = sizeof(MEMORYSTATUSEX);
    GlobalMemoryStatusEx(&memInfo);
    DWORDLONG totalVirtualMem = memInfo.ullTotalPageFile;
	DWORDLONG totalPhysMem = memInfo.ullTotalPhys;
	DWORDLONG physMemUsed = memInfo.ullTotalPhys - memInfo.ullAvailPhys;
	DWORDLONG physMemAvail = memInfo.ullAvailPhys;


    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }

    // create heap storage for depth pixel data in RGBX format
    m_pDepthRGBX = new RGBQUAD[depth_col * depth_row];

	// create heap storage for color pixel data  
	m_pColorRGBX = new RGBQUAD[Color_col * Color_row];
	
	// create heap storage for infrared pixel data in RGBX format
    m_pInfraredRGBX = new RGBQUAD[depth_col * depth_row];

   // create heap storage for composite image pixel data in RGBX format
    m_pOutputRGBX = new RGBQUAD[Color_col * Color_row];

    // create heap storage for background image pixel data in RGBX format
    m_pBackgroundRGBX = new RGBQUAD[Color_col * Color_row];    

	// create heap storage for the coordinate mapping from depth to color
    m_pColorCoordinates = new ColorSpacePoint[depth_col * depth_row];
  
}
  

/// <summary>
/// Destructor
/// </summary>
CCompleteViewer::~CCompleteViewer()
{
    // clean up Direct2D renderer
    if (m_pOutputRGBX)
    {
        delete [] m_pOutputRGBX;
        m_pOutputRGBX = NULL;
    }

    if (m_pBackgroundRGBX)
    {
        delete [] m_pBackgroundRGBX;
        m_pBackgroundRGBX = NULL;
    }

    if (m_pColorRGBX)
    {
        delete [] m_pColorRGBX;
        m_pColorRGBX = NULL;
    }

	if (m_pColorCoordinates)
    {
        delete[] m_pColorCoordinates;
        m_pColorCoordinates = NULL;
    }

    if (m_pDrawDepth)
    {
        delete m_pDrawDepth;
        m_pDrawDepth = NULL;
    }

    if (m_pDepthRGBX)
    {
        delete [] m_pDepthRGBX;
        m_pDepthRGBX = NULL;
    }

    if (m_pDrawColor)
    {
        delete m_pDrawColor;
        m_pDrawColor = NULL;
    }

    if (m_pDrawInfrared)
    {
        delete m_pDrawInfrared;
        m_pDrawInfrared = NULL;
    }

    if (m_pInfraredRGBX)
    {
        delete [] m_pInfraredRGBX;
        m_pInfraredRGBX = NULL;
    }

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

	// done with frame reader
    SafeRelease(m_pMultiSourceFrameReader);

	// done with coordinate mapper
    SafeRelease(m_pCoordinateMapper);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Allocates the arrays we need
/// </summary>
void CCompleteViewer::Allocate()
{

		// allocate dinamic arrays	
		if(checkDepth)
		{
	DepthArrayBuffer = new USHORT*[NumFramesAllocateDepth];
	for(int i = 0; i < NumFramesAllocateDepth; i++)
		DepthArrayBuffer[i] = new USHORT[depth_row*depth_col];
		}

		if(checkColor)
		{
	ColorArrayBuffer = new RGBQUAD*[NumFramesAllocateRGB];
	for(int i = 0; i < NumFramesAllocateRGB; i++)
		ColorArrayBuffer[i] = new RGBQUAD[Color_row*Color_col];
		}

		if(checkIR)
		{
	InfraredArrayBuffer = new RGBQUAD*[NumFramesAllocateIR];
	for(int i = 0; i < NumFramesAllocateIR; i++)
		InfraredArrayBuffer[i] = new RGBQUAD[depth_row*depth_col];
		}

		if(checkSkeleton)
		{
			SkeletonArrayBuffer = new Body[NumFramesAllocateSkel];

			for(int i=0;i<NumFramesAllocateSkel;i++)
			{
				for(int j=0;j<BODY_COUNT;j++)
				{
					for(int k=0;k<JointType_Count;k++)
					{
						SkeletonArrayBuffer[i].body[j][k].x = 0;
						SkeletonArrayBuffer[i].body[j][k].y = 0;
						SkeletonArrayBuffer[i].body[j][k].z = 0;
						SkeletonArrayBuffer[i].body[j][k].w = 9999;
						SkeletonArrayBuffer[i].jointState[j][k] = 9999;
					}
				}
			}

			SkeletonArrayBufferSkSpace = new Body[NumFramesAllocateSkel];

			for(int i=0;i<NumFramesAllocateSkel;i++)
			{
				for(int j=0;j<BODY_COUNT;j++)
				{
					for(int k=0;k<JointType_Count;k++)
					{
						SkeletonArrayBufferSkSpace[i].body[j][k].x = 0;
						SkeletonArrayBufferSkSpace[i].body[j][k].y = 0;
						SkeletonArrayBufferSkSpace[i].body[j][k].z = 0;
						SkeletonArrayBufferSkSpace[i].body[j][k].w = 9999;
						SkeletonArrayBufferSkSpace[i].jointState[j][k] = 9999;
					}
				}
			}

		}

		if(checkMapp)
		{
	MappingArrayBuffer = new USHORT*[NumFramesAllocateMapp];
	for(int i = 0; i < NumFramesAllocateMapp; i++)
		MappingArrayBuffer[i] = new USHORT[depth_row*Mapp_cols];
		}
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CCompleteViewer::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"CompleteViewerAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CCompleteViewer::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    // Main message loop
    while (WM_QUIT != msg.message)
    {
		// Checks if color and depth are selected 
		if(NumFpsCapture == NumFpsCaptureDepth && checkColor && checkDepth && !ClickAllocates)
		{
			EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK5), TRUE);
		}
		if(!checkColor || !checkDepth)  
		{
			checkMapp=false;
			EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK5), FALSE);
			//SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
		}

        Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }

		// Once clicked, disables the button "Save data"
		if(ClickSave)
		{
			EnableWindow(GetDlgItem(m_hWnd, IDC_Save), FALSE);
		}
		
		// checks if all the frames are captured
		if(EndCapture() && ClickAllocates && !ClickSave)
		{
			EnableWindow(GetDlgItem(m_hWnd, IDC_Save), TRUE);
			EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), FALSE);
		}
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CCompleteViewer::Update()
{
	IMultiSourceFrame* pMultiSourceFrame = NULL;
    IDepthFrame* pDepthFrame = NULL;
    IColorFrame* pColorFrame = NULL;
	IBodyFrame* pBodyFrame = NULL;
	IInfraredFrame* pInfraredFrame = NULL;

	if (!m_pMultiSourceFrameReader)
    {
        return;
    }

    HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

    if (SUCCEEDED(hr))
    {
        IDepthFrameReference* pDepthFrameReference = NULL;

        hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
        }

        SafeRelease(pDepthFrameReference);
    }

    if (SUCCEEDED(hr))
    {
        IColorFrameReference* pColorFrameReference = NULL;

        hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);

        if (SUCCEEDED(hr))
        {
            hr = pColorFrameReference->AcquireFrame(&pColorFrame);
        }

        SafeRelease(pColorFrameReference);
    }

	if (SUCCEEDED(hr))

    {
        IBodyFrameReference* pBodyFrameReference = NULL;

        hr = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrameReference->AcquireFrame(&pBodyFrame);
		}

        SafeRelease(pBodyFrameReference);
    }

	if (SUCCEEDED(hr))

    {
        IInfraredFrameReference* pInfraredFrameReference = NULL;

        hr = pMultiSourceFrame->get_InfraredFrameReference(&pInfraredFrameReference);

        if (SUCCEEDED(hr))
        {
            hr = pInfraredFrameReference->AcquireFrame(&pInfraredFrame);
        }

        SafeRelease(pInfraredFrameReference);
    }

	if (SUCCEEDED(hr))
    {
		//Depth
        INT64 nDepthTime = 0;
        IFrameDescription* pDepthFrameDescription = NULL;
        int nDepthWidth = 0;
        int nDepthHeight = 0;
        UINT nDepthBufferSize = 0;
        UINT16 *pDepthBuffer = NULL;

		//Color
		INT64 nColorTime = 0;
        IFrameDescription* pColorFrameDescription = NULL;
        int nColorWidth = 0;
        int nColorHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
        USHORT nDepthMaxDistance = 0;
        ColorImageFormat imageFormat = ColorImageFormat_None;
        UINT nColorBufferSize = 0;
        RGBQUAD *pColorBuffer = NULL;

		//Body
		INT64 nBodyTime = 0;
		IBody* ppBodies[BODY_COUNT] = {0};

		//Infrared
		INT64 nInfraredTime = 0;
        IFrameDescription* pInfraredFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        UINT nBufferSize = 0;
        UINT16 *pinfraredBuffer = NULL;
     

        // get depth frame data
		if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_RelativeTime(&nDepthTime);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameDescription->get_Width(&nDepthWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameDescription->get_Height(&nDepthHeight);

        }

		if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);            
        }

		if (SUCCEEDED(hr))
        {
            // In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nDepthMaxDistance = USHRT_MAX;

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
            //// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);           
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);            
        }	

		if (SUCCEEDED(hr))
        {
            ProcessDepth(nDepthTime, pDepthBuffer, nDepthWidth, nDepthHeight, nDepthMinReliableDistance, nDepthMaxDistance);
        }


        // get color frame data
		if (SUCCEEDED(hr))
        {
            hr = pColorFrame->get_RelativeTime(&nColorTime);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrameDescription->get_Width(&nColorWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrameDescription->get_Height(&nColorHeight);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
        }

        if (SUCCEEDED(hr))
        {
            if (imageFormat == ColorImageFormat_Bgra)
            {
                hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
            }
            else if (m_pColorRGBX)
            {
                pColorBuffer = m_pColorRGBX;
                nColorBufferSize = Color_col * Color_row * sizeof(RGBQUAD);
                hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
            }
            else
            {
                hr = E_FAIL;
            }			
        }

		if (SUCCEEDED(hr))
        {
            ProcessColor(nColorTime, pColorBuffer, nColorWidth, nColorHeight);
        }


		// process mapping
		if (CaptureMappFlag && TotalMappFrameCaptured < NumFramesAllocateMapp)
		{
			ProcessMapp( pDepthBuffer, nDepthWidth, nDepthHeight);
		}


		// get body frame data
		if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->get_RelativeTime(&nBodyTime);
		}

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
        }

        if (SUCCEEDED(hr))
        {
            ProcessBody(nBodyTime, BODY_COUNT, ppBodies);
			
			frameBodyCounter++;
        }

        for (int i = 0; i < _countof(ppBodies); ++i)
        {
            SafeRelease(ppBodies[i]);
        }
		
		
		// get infrared frame data
		if (SUCCEEDED(hr))
        {
		    hr = pInfraredFrame->get_RelativeTime(&nInfraredTime);
		}

        if (SUCCEEDED(hr))
        {
            hr = pInfraredFrame->get_FrameDescription(&pInfraredFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pInfraredFrameDescription->get_Width(&nWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pInfraredFrameDescription->get_Height(&nHeight);
        }

        if (SUCCEEDED(hr))
        {
            hr = pInfraredFrame->AccessUnderlyingBuffer(&nBufferSize, &pinfraredBuffer);            
        }

        if (SUCCEEDED(hr))
        {
            ProcessInfrared(nInfraredTime, pinfraredBuffer, nWidth, nHeight);
        }

		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameDescription);
		SafeRelease(pInfraredFrameDescription);
	}

    SafeRelease(pDepthFrame);
    SafeRelease(pColorFrame); 
	SafeRelease(pBodyFrame);
	SafeRelease(pInfraredFrame);
	SafeRelease(pMultiSourceFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CCompleteViewer::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CCompleteViewer* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CCompleteViewer*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CCompleteViewer*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CCompleteViewer::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
			D2D1CreateFactory(D2D1_FACTORY_TYPE_MULTI_THREADED, &m_pD2DFactory);			

            /// Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen
            m_pDrawColor = new ImageRenderer();
            HRESULT hr = m_pDrawColor->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW2), m_pD2DFactory, Color_col, Color_row, Color_col * sizeof(RGBQUAD)); 
            if (FAILED(hr))
            {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
            }
           
	        // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen
            m_pDrawDepth = new ImageRenderer();
            hr = m_pDrawDepth->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW3), m_pD2DFactory, depth_col, depth_row, depth_col * sizeof(RGBQUAD)); 
            if (FAILED(hr))
            {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
            }

			// Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen
            m_pDrawInfrared = new ImageRenderer();
            hr = m_pDrawInfrared->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW4), m_pD2DFactory, depth_col, depth_row, depth_col * sizeof(RGBQUAD)); 
            if (FAILED(hr))
            {
               SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
            }
        
			//Print path to save
	        SetDlgItemText(m_hWnd, IDC_path, L"C:\\Saved_Data");

			//Print capture fps RGB
	        SetDlgItemText(m_hWnd, IDC_fpsRGB, L"10");

			//Print capture fps Depth
	        SetDlgItemText(m_hWnd, IDC_fpsDepth, L"30");

			//Print capture fps IR
	        SetDlgItemText(m_hWnd, IDC_fpsIR, L"30");

			//Print capture fps Skeleton
	        SetDlgItemText(m_hWnd, IDC_fpsSkeleton, L"30");

			//Print capture time in seconds
	        SetDlgItemText(m_hWnd, IDC_AcquisitionTime, L"1");

            // Get and initialize the default Kinect sensor
            InitializeDefaultSensor();
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
			if(checkColor)					
			{
				for(int i = 0; i < NumFramesAllocateRGB; ++i) 
				delete [] ColorArrayBuffer[i];
				delete [] ColorArrayBuffer;
			}
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

        // Handle button press
        case WM_COMMAND: 
          
			if (IDC_Start == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
			{             
				StartCapture();
			}

			if (IDC_Stop == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
			{
				StopCapture();
			}
			if ( IDC_Allocates == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
			{
				StartAllocates();		
			}
			if (IDC_Save == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
			{
				ClickSave = true;
				Save();
			}

			
			if ( IDC_CHECK6 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))	
			{
				contClickBmp++;
				// check on the button
				// if the button is pressed an odd number of times, the flag is set to true
				if((contClickBmp%2)==0 && ((contClickBmp+contClickPng)%2)==0)
				{
					EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK7), TRUE);
					checkColor=false;
					if(!checkDepth && !checkIR && !checkSkeleton)
					{
						EnableWindow(GetDlgItem(m_hWnd, IDC_Allocates), FALSE);
					}
				}
				else
				{
					EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK7), FALSE);
					checkColor=true;
					EnableWindow(GetDlgItem(m_hWnd, IDC_Allocates), TRUE);
				}
			}


			if ( IDC_CHECK7 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))		
			{
				contClickPng++;
				// check on the button
				// if the button is pressed an odd number of times, the flag is set to true
				if((contClickPng%2 )==0 && ((contClickBmp+contClickPng)%2)==0)
				{
					EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK6), TRUE);		
					checkColor=false;
					if(!checkDepth && !checkIR && !checkSkeleton)
					{
						EnableWindow(GetDlgItem(m_hWnd, IDC_Allocates), FALSE);
					}
				}
				else
				{
					EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK6), FALSE);
					checkColor=true;
					EnableWindow(GetDlgItem(m_hWnd, IDC_Allocates), TRUE);
				}
			}
			

			if ( IDC_CHECK2 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
			{
			contClickIR++;
							// check on the button
				// if the button is pressed an odd number of times, the flag is set to true
				if((contClickIR%2)==0)
				{
					checkIR=false;
					if(!checkDepth && !checkColor && !checkSkeleton)
					{
						EnableWindow(GetDlgItem(m_hWnd, IDC_Allocates), FALSE);
					}
				}
				else
				{
					checkIR=true;
					EnableWindow(GetDlgItem(m_hWnd, IDC_Allocates), TRUE);
				} 
			}

			if ( IDC_CHECK3 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
			{
				contClickDepth++;
								// check on the button
				// if the button is pressed an odd number of times, the flag is set to true
				if((contClickDepth%2)==0)
				{
					checkDepth=false;
					if(!checkColor && !checkIR && !checkSkeleton)
					{
						EnableWindow(GetDlgItem(m_hWnd, IDC_Allocates), FALSE);
					}
				}
				else
				{
					checkDepth=true;
					EnableWindow(GetDlgItem(m_hWnd, IDC_Allocates), TRUE);
				}
			}

			if ( IDC_CHECK4 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
			{
				contClickSkeleton++;
								// check on the button
				// if the button is pressed an odd number of times, the flag is set to true
				if((contClickSkeleton%2)==0)
				{
					checkSkeleton=false;
					if(!checkDepth && !checkIR && !checkColor)
					{
						EnableWindow(GetDlgItem(m_hWnd, IDC_Allocates), FALSE);
					}
				}
				else
				{
					checkSkeleton=true;
					EnableWindow(GetDlgItem(m_hWnd, IDC_Allocates), TRUE);
				}
			}

			if ( IDC_CHECK5 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
			{
				contClickMapp++;
								// check on the button
				// if the button is pressed an odd number of times, the flag is set to true
				if((contClickMapp%2)==0)
				{
					checkMapp=false;
				}
				else
				{
					checkMapp=true;
				}
			}	   
	  break;


	    // update depth frames captured
		case WM_USER_UPDATE_DEPTH:
		{
			SetDlgItemInt( m_hWnd, static_cast<int>(wParam), static_cast<int>(lParam), FALSE );
		}
		break;

		// update body frames captured
		case WM_USER_UPDATE_SKELETON:
		{
			SetDlgItemInt( m_hWnd, static_cast<int>(wParam), static_cast<int>(lParam), FALSE );		
		}
		break;

		// update color frames captured
		case WM_USER_UPDATE_RGB:
		{
			SetDlgItemInt( m_hWnd, static_cast<int>(wParam), static_cast<int>(lParam), FALSE );	
		}
		break;

		// update infrared frames captured
		case WM_USER_UPDATE_IR:
		{
			SetDlgItemInt( m_hWnd, static_cast<int>(wParam), static_cast<int>(lParam), FALSE );
		}
		break;

		// update mapping 
		case WM_USER_UPDATE_MAPP:
		{
			SetDlgItemInt( m_hWnd, static_cast<int>(wParam), static_cast<int>(lParam), FALSE );	
		}
		break;

    }
    return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CCompleteViewer::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {    
        // Initialize the Kinect and get coordinate mapper and the frame reader
        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | 
				FrameSourceTypes::FrameSourceTypes_Body | FrameSourceTypes::FrameSourceTypes_Infrared, &m_pMultiSourceFrameReader);
        }		
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new depth data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// <param name="nMinDepth">minimum reliable depth</param>
/// <param name="nMaxDepth">maximum reliable depth</param>
/// </summary>
void CCompleteViewer::ProcessDepth(INT64 nTime, UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	LARGE_INTEGER qpcNowDepth = {0};
	LARGE_INTEGER actMicrosecs;

	if (m_hWnd)
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		double fps = 0.0;

		LARGE_INTEGER qpcNow = {0};
		if (m_fFreq)
		{
			if (QueryPerformanceCounter(&qpcNow))
			{
				qpcNowDepth = qpcNow;

				if (m_nLastCounter)
				{
					m_nFramesSinceUpdate++;
					fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
				}
			}
		}

		WCHAR szStatusMessage[64];
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

		if (SetStatusMessage(szStatusMessage, 1000, false))
		{
			m_nLastCounter = qpcNow.QuadPart;
			m_nFramesSinceUpdate = 0;
		}			
	}

	// Save the QPC of depth frame
	actMicrosecs.QuadPart = qpcNowDepth.QuadPart;
	// Conversion between ticks and microseconds
	actMicrosecs.QuadPart *= 1000000;
	actMicrosecs.QuadPart /= m_fFreq;

	// Make sure we've received valid data
	if (m_pDepthRGBX && pBuffer && (nWidth == depth_col) && (nHeight == depth_row))
	{
		RGBQUAD* pRGBX = m_pDepthRGBX;
		const USHORT * pBufferDepth = pBuffer;
		const UINT16* pBufferEnd = pBufferDepth + (depth_col * depth_row);
		int cont2 = 0;

		if (CaptureDepthFlag && TotalDepthFrameCaptured < NumFramesAllocateDepth)
		{	
			if(std::floor(30/NumFpsCaptureDepth) == 1)     
				{
					// Save depth time info
					DepthTime[TotalDepthFrameCaptured] = nTime;
					clockTimestampDepth[TotalDepthFrameCaptured] = actMicrosecs;

					int cont = 0;

				while (pBufferDepth < pBufferEnd)
					{
						USHORT depth = *pBufferDepth;
						// Save depth data into the buffer
						DepthArrayBuffer[TotalDepthFrameCaptured][cont] = depth;
						cont++;
						++pBufferDepth;
					}

			// Update counter of frames captured
			PostMessageW( m_hWnd, WM_USER_UPDATE_DEPTH, IDC_depthcaptured, TotalDepthFrameCaptured+1 );
			TotalDepthFrameCaptured++;
				}
			else {	
					if(recfpsDepth == std::floor(30/NumFpsCaptureDepth))                 //// fpsRGB != 30
					{
						DepthTime[TotalDepthFrameCaptured] = nTime;
						clockTimestampDepth[TotalDepthFrameCaptured] = actMicrosecs;

						int cont = 0;

						while (pBufferDepth < pBufferEnd)
							{
								USHORT depth = *pBufferDepth;
								// Save depth data into the buffer
								DepthArrayBuffer[TotalDepthFrameCaptured][cont] = depth;
								cont++;
								++pBufferDepth;
							}

						// Update counter of frames captured
						PostMessageW( m_hWnd, WM_USER_UPDATE_DEPTH, IDC_depthcaptured, TotalDepthFrameCaptured+1 );
						TotalDepthFrameCaptured++;
						recfpsDepth = recfpsDepth--;
					}
			else if(recfpsDepth == 1 )
					{
						recfpsDepth = std::floor(30/NumFpsCaptureDepth);                   
					}
			else recfpsDepth--;
				}				
		}  
		while (pBuffer < pBufferEnd)                  
			{
				USHORT depth = *pBuffer;
				FrameAct[cont2] =depth; 
				cont2++;
				// To convert to a byte, we're discarding the most-significant
				// rather than least-significant bits.
				// We're preserving detail, although the intensity will "wrap."
				// Values outside the reliable depth range are mapped to 0 (black).

				// Note: Using conditionals in this loop could degrade performance.
				// Consider using a lookup table instead when writing production code.
				BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

				pRGBX->rgbRed   = intensity;
				pRGBX->rgbGreen = intensity;
				pRGBX->rgbBlue  = intensity;

				++pRGBX;
				++pBuffer;
			}
		m_pDrawDepth->Draw(reinterpret_cast<BYTE*>(m_pDepthRGBX), depth_col * depth_row * sizeof(RGBQUAD));
	}
}

/// <summary>
/// Handle new color data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// </summary>
void CCompleteViewer::ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight) 
{
	LARGE_INTEGER qpcNowRGB = {0};
	LARGE_INTEGER actMicrosecs;
	//int actBytIdx = 0;		

	if (m_hWnd)
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		LARGE_INTEGER qpcNow = {0}; 
		if (m_fFreq)
		{
			if (QueryPerformanceCounter(&qpcNow))
			{
				qpcNowRGB = qpcNow;
			}
		}
	}

	// Save the QPC of color frame
	actMicrosecs.QuadPart = qpcNowRGB.QuadPart;
	// Conversion between ticks and microseconds
	actMicrosecs.QuadPart *= 1000000;
	actMicrosecs.QuadPart /= m_fFreq;

	// Make sure we've received valid data
	if (pBuffer && (nWidth == Color_col) && (nHeight == Color_row))
	{  

		if (CaptureColorFlag && TotalColorFrameCaptured < NumFramesAllocateRGB)
		{

			if(std::floor(30/NumFpsCapture) == 1)      //fpsRGB = 30
			{
				ColorTime[TotalColorFrameCaptured] = nTime;
				clockTimestampColor[TotalColorFrameCaptured] = actMicrosecs;

				// Save Color data into the buffer
				for(int val = 0; val < (Color_row*Color_col); val++) 
				{	
					ColorArrayBuffer[TotalColorFrameCaptured][val] = pBuffer[val];		
				}

				// Update counter of frames captured
				PostMessageW( m_hWnd, WM_USER_UPDATE_RGB, IDC_rgbcaptured, TotalColorFrameCaptured+1 );
				TotalColorFrameCaptured++;
			}
		else {	
				if(recfps == std::floor(30/NumFpsCapture))                 //fpsRGB != 30
					{
						ColorTime[TotalColorFrameCaptured] = nTime;
						clockTimestampColor[TotalColorFrameCaptured] = actMicrosecs;

						// Save Color data into the buffer
						for(int val = 0; val < (Color_row*Color_col); val++)  
							{	
								ColorArrayBuffer[TotalColorFrameCaptured][val] = pBuffer[val];
							}
						// Update counter of frames captured
						PostMessageW( m_hWnd, WM_USER_UPDATE_RGB, IDC_rgbcaptured, TotalColorFrameCaptured+1 );    
						TotalColorFrameCaptured++;
						recfps = recfps--;
					}
				else if(recfps == 1 )
					{
						recfps = std::floor(30/NumFpsCapture);                   
					}
				else recfps--;
			}
		}
	}	

	//Draw the data with Direct2D
	m_pDrawColor->Draw(reinterpret_cast<BYTE*>(pBuffer), Color_col * Color_row * sizeof(RGBQUAD)); //serve per proiettare l’immagine di colore nella finestra dell’interfaccia grafica ed avere così la visuale del sensore
}

/// <summary>
/// Create and save the mapping matrix into the buffer
/// <param name="pDepthBuffer">pointer to frame data</param>
/// <param name="nDepthWidth">width (in pixels) of input image data</param>
/// <param name="nDepthHeight">height (in pixels) of input image data</param>
/// </summary>
void CCompleteViewer::ProcessMapp(UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight)
{
	// Make sure we've received valid data
	if (m_pCoordinateMapper && m_pColorCoordinates && pDepthBuffer && (nDepthWidth == depth_col) && (nDepthHeight == depth_row))
	{
		HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, nDepthWidth * nDepthHeight, m_pColorCoordinates);
		if (SUCCEEDED(hr))
		{
			if (CaptureMappFlag && TotalMappFrameCaptured < NumFramesAllocateMapp)
			{	
				if(std::floor(30/NumFpsCaptureMapp) == 1)      ////fpsRGB = 30  
					{
						int val = 0;
						for( int y = 0; y < depth_row; y++ )
						{
							for( int x = 0; x < depth_col; x++ )
							{
								unsigned int index = y * depth_col + x;
								ColorSpacePoint point = m_pColorCoordinates[index];
								int colorX = static_cast<int>( std::floor( point.X + 0.5 ) );
								int colorY = static_cast<int>( std::floor( point.Y + 0.5 ) );
								if( ( colorX >= 0 ) && ( colorX < Color_col ) && ( colorY >= 0 ) && ( colorY < Color_row ))
								{
									MappingArrayBuffer[TotalMappFrameCaptured][val] = colorY;
									MappingArrayBuffer[TotalMappFrameCaptured][val+1] = colorX;
								}
								val = val + 2;
							}
						}
						// Update counter of mapping
						PostMessageW( m_hWnd, WM_USER_UPDATE_MAPP, IDC_mappcaptured, TotalMappFrameCaptured+1 );
						TotalMappFrameCaptured++;
					}
				else {	
						if(recfpsMapp == std::floor(30/NumFpsCaptureMapp))                 //// fpsRGB != 30
							{
								int val = 0;
								for( int y = 0; y < depth_row; y++ )
									{
										for( int x = 0; x < depth_col; x++ )
											{
												unsigned int index = y * depth_col + x;
												ColorSpacePoint point = m_pColorCoordinates[index];
												int colorX = static_cast<int>( std::floor( point.X + 0.5 ) );
												int colorY = static_cast<int>( std::floor( point.Y + 0.5 ) );
												if( ( colorX >= 0 ) && ( colorX < Color_col ) && ( colorY >= 0 ) && ( colorY < Color_row ))
													{
														MappingArrayBuffer[TotalMappFrameCaptured][val] = colorY;
														MappingArrayBuffer[TotalMappFrameCaptured][val+1] = colorX;
													}
												val = val + 2;
											}
									}
								// Update counter of mapping
								PostMessageW( m_hWnd, WM_USER_UPDATE_MAPP, IDC_mappcaptured, TotalMappFrameCaptured+1 );
								TotalMappFrameCaptured++;
								recfpsMapp = recfpsMapp--;
							}
						else if(recfpsMapp == 1 )
							{
								recfpsMapp = std::floor(30/NumFpsCaptureMapp);                   
							}
						else recfpsMapp--;
				}
			}
		}
	}
}

/// <summary>
/// Handle new infrared data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// </summary>
void CCompleteViewer::ProcessInfrared(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight)
{
	LARGE_INTEGER qpcNowIR = {0};
	LARGE_INTEGER actMicrosecs;

	if (m_hWnd)
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		//double fps = 0.0;

		LARGE_INTEGER qpcNow = {0};
		if (m_fFreq)
		{
			if (QueryPerformanceCounter(&qpcNow))
			{
				qpcNowIR = qpcNow; 
			}
		}
	}

	// Save the QPC of infrared frame
	actMicrosecs.QuadPart = qpcNowIR.QuadPart;
	// Conversion between ticks and microseconds
	actMicrosecs.QuadPart *= 1000000;
	actMicrosecs.QuadPart /= m_fFreq;


	if (m_pInfraredRGBX && pBuffer && (nWidth == depth_col) && (nHeight == depth_row))
	{
		RGBQUAD* pRGBX = m_pInfraredRGBX;
			const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);


		if ( CaptureIRFlag && TotalIRFrameCaptured < NumFramesAllocateIR)
		{
			if(std::floor(30/NumFpsCaptureIR) == 1)      ////fpsIR = 30  		
				{//Save infrared time info
					InfraredTime[TotalIRFrameCaptured] = nTime;
					clockTimestampIR[TotalIRFrameCaptured] = actMicrosecs;

					while (pBuffer < pBufferEnd)
						{
							USHORT ir = *pBuffer;
							// To convert to a byte, we're discarding the least-significant bits.
							BYTE intensity = static_cast<BYTE>(ir >> 8);

							pRGBX->rgbRed   = intensity;
							pRGBX->rgbGreen = intensity;
							pRGBX->rgbBlue  = intensity;

							++pRGBX;
							++pBuffer;
						}

					for(int val = 0; val < (depth_row*depth_col); val++)
						{	
							// Save infrared data into the buffer
							InfraredArrayBuffer[TotalIRFrameCaptured][val] = m_pInfraredRGBX[val];
						}

					// Update counter of frames captured
					PostMessageW( m_hWnd, WM_USER_UPDATE_IR, IDC_ircaptured, TotalIRFrameCaptured+1);
					TotalIRFrameCaptured++;
				}
			else {	
					if(recfpsIR == std::floor(30/NumFpsCaptureIR))                 //// fpsIR != 30
						{
							InfraredTime[TotalIRFrameCaptured] = nTime;
							clockTimestampIR[TotalIRFrameCaptured] = actMicrosecs;

							while (pBuffer < pBufferEnd)
								{
									USHORT ir = *pBuffer;
									// To convert to a byte, we're discarding the least-significant bits.
									BYTE intensity = static_cast<BYTE>(ir >> 8);

									pRGBX->rgbRed   = intensity;
									pRGBX->rgbGreen = intensity;
									pRGBX->rgbBlue  = intensity;
	
									++pRGBX;
									++pBuffer;
								}

							for(int val = 0; val < (depth_row*depth_col); val++)
								{	
									// Save infrared data into the buffer
									InfraredArrayBuffer[TotalIRFrameCaptured][val] = m_pInfraredRGBX[val];
								}

							// Update counter of frames captured
							PostMessageW( m_hWnd, WM_USER_UPDATE_IR, IDC_ircaptured, TotalIRFrameCaptured+1);
							TotalIRFrameCaptured++;
							recfpsIR = recfpsIR--;
						}
					else if(recfpsIR == 1 )
						{
							recfpsIR = std::floor(30/NumFpsCaptureIR);                   
						}
					else recfpsIR--;
				}
		}
		while (pBuffer < pBufferEnd)
			{
				USHORT ir = *pBuffer;
				// To convert to a byte, we're discarding the least-significant bits.
				BYTE intensity = static_cast<BYTE>(ir >> 8);

				pRGBX->rgbRed   = intensity;
				pRGBX->rgbGreen = intensity;
				pRGBX->rgbBlue  = intensity;

				++pRGBX;
				++pBuffer;
			}

			// Draw the data with Direct2D
			m_pDrawInfrared->Draw(reinterpret_cast<BYTE*>(m_pInfraredRGBX), depth_col * depth_row * sizeof(RGBQUAD));		
	}
}

/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// </summary>
void CCompleteViewer::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies)
{
	LARGE_INTEGER qpcNowBody = {0};
	LARGE_INTEGER actMicrosecs;

	if (m_hWnd)
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		//double fps = 0.0;

		LARGE_INTEGER qpcNow = {0};
		if (m_fFreq)
		{
			if (QueryPerformanceCounter(&qpcNow))
			{
				qpcNowBody = qpcNow;
			}
		}
	}

	// Save the QPC of depth frame
	actMicrosecs.QuadPart = qpcNowBody.QuadPart;
	// Conversion between ticks and microseconds
	actMicrosecs.QuadPart *= 1000000;
	actMicrosecs.QuadPart /= m_fFreq;

	HRESULT hr = EnsureDirect2DResources();

	if (SUCCEEDED(hr) && m_pRenderTarget && m_pCoordinateMapper)
		{
			m_pRenderTarget->BeginDraw();
			m_pRenderTarget->Clear();

			RECT rct;
			GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
			int width = rct.right;
			int height = rct.bottom;

			for (int i = 0; i < nBodyCount; ++i)
			{
				IBody* pBody = ppBodies[i];
				if (pBody)
				{
					BOOLEAN bTracked = false;
					hr = pBody->get_IsTracked(&bTracked);

					if (SUCCEEDED(hr) && bTracked)
					{
						Joint joints[JointType_Count]; 
						D2D1_POINT_2F jointPoints[JointType_Count];
						HandState leftHandState = HandState_Unknown;
						HandState rightHandState = HandState_Unknown;

						pBody->get_HandLeftState(&leftHandState);
						pBody->get_HandRightState(&rightHandState);

						hr = pBody->GetJoints(_countof(joints), joints);
						if (SUCCEEDED(hr))
						{
							if(CaptureSkeletonFlag && TotalSkeletonFrameCaptured < NumFramesAllocateSkel)
							{
								if(std::floor(30/NumFpsCaptureSkeleton) == 1)      ////fpsSkeleton = 30  		
								{
									BodyTime[TotalSkeletonFrameCaptured] = nTime;
									clockTimestampBody[TotalSkeletonFrameCaptured] = actMicrosecs;
									for (int j = 0; j < _countof(joints); ++j)
									{
										jointPoints[j] = BodyToScreen(joints[j].Position, joints[j].TrackingState , width, height, i, j);
									}
								}	
								else
								{	
									if(recfpsSkeleton == std::floor(30/NumFpsCaptureSkeleton))   
									{
										BodyTime[TotalSkeletonFrameCaptured] = nTime;
										clockTimestampBody[TotalSkeletonFrameCaptured] = actMicrosecs;	
										for (int j = 0; j < _countof(joints); ++j)
										{
											jointPoints[j] = BodyToScreen(joints[j].Position, joints[j].TrackingState , width, height, i, j);
										}
										recfpsSkeleton = recfpsSkeleton--;
									}
									else if(recfpsSkeleton == 1 )
									{
										recfpsSkeleton = std::floor(30/NumFpsCaptureSkeleton);               
									}
									else 
									{	
										recfpsSkeleton--;
									}
								}
							}

							for (int j = 0; j < _countof(joints); ++j)
							{
								jointPoints[j] = BodyToScreenPlot(joints[j].Position, joints[j].TrackingState , width, height);
							}
							
							DrawBody(joints, jointPoints);
							DrawHand(leftHandState, jointPoints[JointType_HandLeft]);
							DrawHand(rightHandState, jointPoints[JointType_HandRight]);
						}
					}
				}
			
			}
			
			hr = m_pRenderTarget->EndDraw();
			// Device lost, need to recreate the render target
			// We'll dispose it now and retry drawing
			if (D2DERR_RECREATE_TARGET == hr)
			{
				hr = S_OK;
				DiscardDirect2DResources();
			}

		}
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>S_OK if successful, otherwise an error code</returns>
HRESULT CCompleteViewer::EnsureDirect2DResources()
{
    HRESULT hr = S_OK;

    if (m_pD2DFactory && !m_pRenderTarget)
    {
        RECT rc;
        GetWindowRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rc);  

        int width = rc.right - rc.left;
        int height = rc.bottom - rc.top;
        D2D1_SIZE_U size = D2D1::SizeU(width, height);
        D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
        rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

        // Create a Hwnd render target, in order to render to the window set in initialize
        hr = m_pD2DFactory->CreateHwndRenderTarget( rtProps, D2D1::HwndRenderTargetProperties(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), size), &m_pRenderTarget);

        if (FAILED(hr))
        {
            SetStatusMessage(L"Couldn't create Direct2D render target!", 10000, true);
            return hr;
        }

        // light green
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pBrushJointInferred);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Gray, 1.0f), &m_pBrushBoneInferred);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red, 0.5f), &m_pBrushHandClosed);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 0.5f), &m_pBrushHandOpen);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Blue, 0.5f), &m_pBrushHandLasso);
    }

    return hr;
}

/// <summary>
/// Dispose Direct2d resources 
/// </summary>
void CCompleteViewer::DiscardDirect2DResources()
{
    SafeRelease(m_pRenderTarget);
    SafeRelease(m_pBrushJointTracked);
    SafeRelease(m_pBrushJointInferred);
    SafeRelease(m_pBrushBoneTracked);
    SafeRelease(m_pBrushBoneInferred);
    SafeRelease(m_pBrushHandClosed);
    SafeRelease(m_pBrushHandOpen);
    SafeRelease(m_pBrushHandLasso);
}

/// <summary>
/// Converts a body point to screen space and save body in a buffer
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="jointState">the joint state</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <param name="i">index of body captured</param>
/// <param name="j">index of joint captured</param>
D2D1_POINT_2F CCompleteViewer::BodyToScreen(const CameraSpacePoint& bodyPoint, TrackingState jointState, int width, int height, int i, int j)
{

	LONG x, y;
	USHORT depth;
	// Calculate the body's position on the screen
	DepthSpacePoint depthPoint = {0};
	m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

	float screenPointX = static_cast<float>(depthPoint.X * width) / depth_col;		
	float screenPointY = static_cast<float>(depthPoint.Y * height) / depth_row;		

	x=depthPoint.X;		
	y=depthPoint.Y;		

	if(CaptureSkeletonFlag && TotalSkeletonFrameCaptured < NumFramesAllocateSkel)
	{
		Vector4 App;
		Vector4 App2;

		// Check for avoid error
		if ((y>depth_row-1) || (x>depth_col-1) || (y<0) || (x<0))
		{
			depth = 9999;               //9999 is an invalid data

			// Coordinates in depth space
			App.x = depth;						
			App.y = depth;
			App.z = depth;
			App.w = i;  
		}
		else
		{
			depth = FrameAct[y*depth_col+x];

			// Coordinates in depth space
			App.x = depthPoint.X;						
			App.y = depthPoint.Y;
			App.z = depth;
			App.w = i;                      
		}

		// Coordinates in skeleton space
		App2.x = bodyPoint.X*1000;						
		App2.y = bodyPoint.Y*1000;
		App2.z = bodyPoint.Z*1000 ;
		App2.w = i;  

			// Save body data into the buffer
		SkeletonArrayBuffer[TotalSkeletonFrameCaptured].body[i][j] = App;				         // Matrix of joints in depth space 
		SkeletonArrayBufferSkSpace[TotalSkeletonFrameCaptured].body[i][j] = App2;		         // Matrix of joints in skeleton space
		SkeletonArrayBuffer[TotalSkeletonFrameCaptured].jointState[i][j] = jointState;  
		
		
		if(j==24 && (temp==0 || temp<frameBodyCounter))
			{
				temp = frameBodyCounter;
				// Update counter of frames captured
				PostMessageW( m_hWnd, WM_USER_UPDATE_SKELETON, IDC_bodycaptured, TotalSkeletonFrameCaptured+1 );  
				if(TotalSkeletonFrameCaptured < NumFramesAllocateSkel)
					{
						TotalSkeletonFrameCaptured++;    
					}
			}
	
	}

	return D2D1::Point2F(screenPointX, screenPointY);	
}


/// <summary>
/// Converts a body point to screen space and save body in a buffer
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="jointState">the joint state</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
D2D1_POINT_2F CCompleteViewer::BodyToScreenPlot(const CameraSpacePoint& bodyPoint, TrackingState jointState, int width, int height)
{
	// Calculate the body's position on the screen
	DepthSpacePoint depthPoint = {0};
	m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

	float screenPointX = static_cast<float>(depthPoint.X * width) / depth_col;		
	float screenPointY = static_cast<float>(depthPoint.Y * height) / depth_row;		

	return D2D1::Point2F(screenPointX, screenPointY);		
}


/// <summary>
/// Draws a body 
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
void CCompleteViewer::DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints)
{
    // Draw the bones

    // Torso
    DrawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck);
    DrawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid);
    DrawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft);
    DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight);
    DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft);
    
    // Right Arm    
    DrawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight);
    DrawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight);
    DrawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight);

    // Left Arm
    DrawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft);
    DrawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft);
    DrawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft);

    // Right Leg
    DrawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight);
    DrawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight);
    DrawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight);

    // Left Leg
    DrawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft);
    DrawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft);
    DrawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft);

    // Draw the joints
    for (int i = 0; i < JointType_Count; ++i)
    {
        D2D1_ELLIPSE ellipse = D2D1::Ellipse(pJointPoints[i], c_JointThickness, c_JointThickness);

        if (pJoints[i].TrackingState == TrackingState_Inferred)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointInferred);
        }
        else if (pJoints[i].TrackingState == TrackingState_Tracked)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
        }
    }
}

/// <summary>
/// Draws one bone of a body (joint to joint)
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="joint0">one joint of the bone to draw</param>
/// <param name="joint1">other joint of the bone to draw</param>
void CCompleteViewer::DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1)
{
    TrackingState joint0State = pJoints[joint0].TrackingState;
    TrackingState joint1State = pJoints[joint1].TrackingState;

    // If we can't find either of these joints, exit
    if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
    {
        return;
    }

    // Don't draw if both points are inferred
    if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
    {
        return;
    }

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
    {
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness);
    }
    else
    {
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneInferred, c_InferredBoneThickness);
    }
}

/// <summary>
/// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
/// </summary>
/// <param name="handState">state of the hand</param>
/// <param name="handPosition">position of the hand</param>
void CCompleteViewer::DrawHand(HandState handState, const D2D1_POINT_2F& handPosition)
{
    D2D1_ELLIPSE ellipse = D2D1::Ellipse(handPosition, c_HandSize, c_HandSize);

    switch (handState)
    {
        case HandState_Closed:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandClosed);
            break;

        case HandState_Open:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandOpen);
            break;

        case HandState_Lasso:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandLasso);
            break;
    }
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CCompleteViewer::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    DWORD now = GetTickCount();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;
        return true;
    }

    return false;
}

/// <summary>
/// Save passed in image data to disk as a bitmap
/// </summary>
/// <param name="pBitmapBits">image data to save</param>
/// <param name="lWidth">width (in pixels) of input image data</param>
/// <param name="lHeight">height (in pixels) of input image data</param>
/// <param name="wBitsPerPixel">bits per pixel of image data</param>
/// <param name="lpszFilePath">full file path to output bitmap to</param>
/// <returns>indicates success or failure</returns>
HRESULT SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{
	DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

    BITMAPINFOHEADER bmpInfoHeader = {0};		//struttura definita da windows

    bmpInfoHeader.biSize        = sizeof(BITMAPINFOHEADER);  // Size of the header
    bmpInfoHeader.biBitCount    = wBitsPerPixel;             // Bit count
    bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression                    
    bmpInfoHeader.biWidth       = lWidth;                    // Width in pixels
    bmpInfoHeader.biHeight      = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
    bmpInfoHeader.biPlanes      = 1;                         // Default
    bmpInfoHeader.biSizeImage   = dwByteCount;               // Image size in bytes

    BITMAPFILEHEADER bfh = {0};

    bfh.bfType    = 0x4D42;        //0x89504e47; png big endian //0x474E5089; png little endian                                 // 'M''B', indicates bitmap
    bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
    bfh.bfSize    = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

    // Create the file on disk to write to
    HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

	std::string pBmpStr((char*)pBitmapBits);
	int thesize = sizeof(pBmpStr); 

    // Return if error opening file
    if (NULL == hFile) 
    {
        return E_ACCESSDENIED;
    }

    DWORD dwBytesWritten = 0;
    
    // Write the bitmap file header
    if (!WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the bitmap info header
    if (!WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the RGB Data
    if (!WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }    

    // Close the file
    CloseHandle(hFile);
    return S_OK;
}



/// <summary>
/// Thread that saves all the data to disk
/// </summary>
unsigned  __stdcall SaveThread(void * m_hWnd)
{
	HANDLE myhandle[3];
	std::set<HANDLE> thread_handles; 
	std::vector<HANDLE> ary;         // WaitForMultipleObjects wants an array...

	if(ThreadsSupported == 1)
	{
		IntervalThread1 = TotalColorFrameCaptured;		//end of first thread
	}
	else if(ThreadsSupported == 2)
	{
		IntervalThread1 = ceil(TotalColorFrameCaptured/2);		//end of first thread and start of last thread
		myhandle[0] = (HANDLE)_beginthreadex(0, 0, &SaveThread2, 0, 0, 0);
		thread_handles.insert(myhandle[0]);	
	}
	else if(ThreadsSupported == 4)
	{	//4 threads
		IntervalThread1 = floor(floor(TotalColorFrameCaptured/2)/2);		//end of first thread and start of second
		IntervalThread2 = floor(TotalColorFrameCaptured/2);		//end of second thread and start of third
		IntervalThread3 = floor(TotalColorFrameCaptured/2) + floor(ceil(TotalColorFrameCaptured/2)/2);		//end of third thread and start of fourth
			
		myhandle[0] = (HANDLE)_beginthreadex(0, 0, &SaveThread2, 0, 0, 0);
		myhandle[1] = (HANDLE)_beginthreadex(0, 0, &SaveThread3, 0, 0, 0);
		myhandle[2] = (HANDLE)_beginthreadex(0, 0, &SaveThread4, 0, 0, 0);
			
		thread_handles.insert(myhandle[0]);
		thread_handles.insert(myhandle[1]);
		thread_handles.insert(myhandle[2]);

	}

	if(checkDepth)
	{
		// Save depth time
		StringCchPrintfW(TimeFileName,_countof(TimeFileName),L"%s\\DepthTime.csv", FolderName_t);	

		FileDepthTime.open(TimeFileName, ios::out);

		for (int i=0; i<TotalDepthFrameCaptured; i++)
		{	
			// Save nTime + QPC Timestamps
			FileDepthTime<<DepthTime[i]<<","<<clockTimestampDepth[i].QuadPart<<","<<endl; 
		}
		FileDepthTime.close();

		// Save depth data
		for(int StartIndex = 0; StartIndex < TotalDepthFrameCaptured; StartIndex++)
		{
			StringCchPrintfW(DataFileName,_countof(DataFileName),L"%s\\Filedepth_%d.bin", FolderName_d, StartIndex);	
		
			DWORD dwBytesWritten = 0;

			// Create the file on disk to write to
            HANDLE hFile = CreateFileW(DataFileName, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

			// Write the Depth Data
            WriteFile(hFile, *(&DepthArrayBuffer[StartIndex]), (depth_row*depth_col)*sizeof( USHORT ), &dwBytesWritten, NULL);

			// Close the file
			CloseHandle(hFile);
			
			// Update counter of frames saved
			PostMessageW( reinterpret_cast <HWND>(m_hWnd), WM_USER_UPDATE_DEPTH, IDC_depthsaved, StartIndex+1 );
		}

		// Deallocation
		for(int i = 0; i < NumFramesAllocateDepth; ++i) 
			delete [] DepthArrayBuffer[i];
		delete [] DepthArrayBuffer;
	}

	if(checkSkeleton)
	{	
		//Save Body time
		StringCchPrintfW(TimeFileName,_countof(TimeFileName),L"%s\\BodyTime.csv", FolderName_t);

		FileBodyTime.open(TimeFileName, ios::out);

		for (int i=0; i<TotalSkeletonFrameCaptured; i++)
		{	
			// Save nTime + QPC Timestamps
			FileBodyTime<<BodyTime[i]<<","<<clockTimestampBody[i].QuadPart<<","<<endl; 
		}

		FileBodyTime.close();

		StringCchPrintfW(FileNameSk,_countof(FileNameSk),L"%s\\Fileskeleton.txt", FolderName_b);
		StringCchPrintfW(FileNameSkCsv,_countof(FileNameSkCsv),L"%s\\Fileskeleton.csv", FolderName_b);
		StringCchPrintfW(FileNameSkSpace,_countof(FileNameSkSpace),L"%s\\FileskeletonSkSpace.txt", FolderName_b);
		StringCchPrintfW(FileNameSkSpaceCsv,_countof(FileNameSkSpaceCsv),L"%s\\FileskeletonSkSpace.csv", FolderName_b);

		fileskeleton.open(FileNameSk, ios::out);
		fileskeletonCsv.open(FileNameSkCsv, ios::out);
		fileskeletonSkSpace.open(FileNameSkSpace, ios::out);
		fileskeletonSkSpaceCsv.open(FileNameSkSpaceCsv, ios::out);

		// Save body data
		for (int StartIndex=0; StartIndex<TotalSkeletonFrameCaptured; StartIndex++)
		{
			fileskeleton<<"Frame "<<StartIndex<<endl;
			fileskeletonSkSpace<<"Frame "<<StartIndex<<endl;

			for(int b=0; b<BODY_COUNT; b++)
			{
				fileskeleton<<"x"<<"\t\t"<<"y"<<"\t\t"<<"z"<<"\t\t"<<"State"<<"\t\t"<<"PlayerIndex"<<endl;
				fileskeletonSkSpace<<"x"<<"\t\t"<<"y"<<"\t\t"<<"z"<<"\t\t"<<"State"<<"\t\t"<<"PlayerIndex"<<endl;

				for(int i=0; i<JointType_Count; i++)
				{
					// ".txt" files
					fileskeleton<<SkeletonArrayBuffer[StartIndex].body[b][i].x<<"\t\t"<<SkeletonArrayBuffer[StartIndex].body[b][i].y<<"\t\t"<<SkeletonArrayBuffer[StartIndex].body[b][i].z<<"\t\t"<<SkeletonArrayBuffer[StartIndex].jointState[b][i]<<"\t\t"<<SkeletonArrayBuffer[StartIndex].body[b][i].w<<endl;
					fileskeletonSkSpace<<SkeletonArrayBufferSkSpace[StartIndex].body[b][i].x<<"\t\t"<<SkeletonArrayBufferSkSpace[StartIndex].body[b][i].y<<"\t\t"<<SkeletonArrayBufferSkSpace[StartIndex].body[b][i].z<<"\t\t"<<SkeletonArrayBuffer[StartIndex].jointState[b][i]<<"\t\t"<<SkeletonArrayBufferSkSpace[StartIndex].body[b][i].w<<endl;

					// ".csv" files
					fileskeletonCsv<<SkeletonArrayBuffer[StartIndex].body[b][i].x<<","<<SkeletonArrayBuffer[StartIndex].body[b][i].y<<","<<SkeletonArrayBuffer[StartIndex].body[b][i].z<<","<<SkeletonArrayBuffer[StartIndex].jointState[b][i]<<","<<SkeletonArrayBuffer[StartIndex].body[b][i].w<<endl;
					fileskeletonSkSpaceCsv<<SkeletonArrayBufferSkSpace[StartIndex].body[b][i].x<<","<<SkeletonArrayBufferSkSpace[StartIndex].body[b][i].y<<","<<SkeletonArrayBufferSkSpace[StartIndex].body[b][i].z<<","<<SkeletonArrayBuffer[StartIndex].jointState[b][i]<<","<<SkeletonArrayBufferSkSpace[StartIndex].body[b][i].w<<endl;
				}
			}

			// Update counter of frames saved
			PostMessageW( reinterpret_cast <HWND>(m_hWnd), WM_USER_UPDATE_SKELETON, IDC_bodysaved, StartIndex+1 );
		}

		fileskeleton.close();
		fileskeletonSkSpace.close();
		fileskeletonCsv.close();
		fileskeletonSkSpaceCsv.close();
	}	

	if(checkIR)
	{
		// Save Infrared time
		StringCchPrintfW(TimeFileName,_countof(TimeFileName),L"%s\\InfraredTime.csv", FolderName_t);

		FileInfraredTime.open(TimeFileName, ios::out);

		for (int i=0; i<TotalIRFrameCaptured; i++)
		{		
			// Save nTime + QPC Timestamps	
			FileInfraredTime<<InfraredTime[i]<<","<<clockTimestampIR[i].QuadPart<<","<<endl; 
		}

		FileInfraredTime.close();

		// Save Infrared data
		for(int StartIndex = 0; StartIndex < TotalIRFrameCaptured; StartIndex++)
		{
			StringCchPrintfW(DataFileName,_countof(DataFileName),L"%s\\KinectScreenshot_IR%d.bmp", FolderName_i, StartIndex);
			SaveBitmapToFile(reinterpret_cast <BYTE*>(InfraredArrayBuffer[StartIndex]), depth_col, depth_row, sizeof(RGBQUAD) * 8, DataFileName);

			// Update counter of frames saved
			PostMessageW( reinterpret_cast <HWND>(m_hWnd), WM_USER_UPDATE_IR, IDC_irsaved, StartIndex+1 );
		}

		// Deallocation
		for(int i = 0; i < NumFramesAllocateIR; ++i) 
			delete [] InfraredArrayBuffer[i];
		delete [] InfraredArrayBuffer;
	}

	if(checkMapp)
	{
		// Save mapping data
		for(int StartIndex = 0; StartIndex < TotalMappFrameCaptured; StartIndex++)
		{
			StringCchPrintfW(DataFileName,_countof(DataFileName),L"%s\\FileMapp_%d.bin", FolderName_m, StartIndex);

			DWORD dwBytesWritten = 0;

			 // Create the file on disk to write to
             HANDLE hFile = CreateFileW(DataFileName, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

			// Write the Mapping Data
            WriteFile(hFile, *(&MappingArrayBuffer[StartIndex]), (depth_row*Mapp_cols)*sizeof( USHORT ), &dwBytesWritten, NULL);

			// Close the file
			CloseHandle(hFile);
			
			// Update counter of frames saved
			PostMessageW( reinterpret_cast <HWND>(m_hWnd), WM_USER_UPDATE_MAPP, IDC_mappsaved, StartIndex+1 );
		}

		// Deallocation
		for(int i = 0; i < NumFramesAllocateMapp; ++i) 
			delete [] MappingArrayBuffer[i];
		delete [] MappingArrayBuffer;		
	}

	if(checkColor)
	{

		if((contClickBmp%2)==1)			
		{
			//Save color time
			StringCchPrintfW(TimeFileName,_countof(TimeFileName),L"%s\\ColorTime.csv", FolderName_t);

			FileColorTime.open(TimeFileName, ios::out);

			for (int i=0; i< TotalColorFrameCaptured; i++)
			{		
				// Save nTime + QPC Timestamps	
				FileColorTime<<ColorTime[i]<<","<<clockTimestampColor[i].QuadPart<<","<<endl; //nTime + QPC Timestamps		
			}

			FileColorTime.close();

			SetDlgItemText(reinterpret_cast <HWND>(m_hWnd), IDC_rgbsaved, _T("Saving "));
			// Save color data
			for(int StartIndex = 0; StartIndex < IntervalThread1; StartIndex++)
			{
				StringCchPrintfW(DataFileName,_countof(DataFileName),L"%s\\KinectScreenshot_RGB%d.bmp", FolderName_c, StartIndex);	
				SaveBitmapToFile(reinterpret_cast <BYTE*>(ColorArrayBuffer[StartIndex]), Color_col, Color_row, sizeof(RGBQUAD) * 8, DataFileName);
			}
		}

		if((contClickPng%2)==1)			
		{
			Mat mat(1080, 1920, CV_8UC4);

			vector<int> compression_params;
			compression_params.push_back(IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(9);

			RGBQUAD* pBuffer;

			//Save color time
			StringCchPrintfW(TimeFileName,_countof(TimeFileName),L"%s\\ColorTime.csv", FolderName_t);

			FileColorTime.open(TimeFileName, ios::out);

			for (int i=0; i< TotalColorFrameCaptured; i++)
			{		
				// Save nTime + QPC Timestamps	
				FileColorTime<<ColorTime[i]<<","<<clockTimestampColor[i].QuadPart<<","<<endl; //nTime + QPC Timestamps		
			}

			FileColorTime.close();

			char pngdirectory[MAX_PATH]; 
			_bstr_t b(FolderName_png);				
			const char* Foldername_pngchar = b;
			printf("%s", Foldername_pngchar);
			strcpy(pngdirectory,Foldername_pngchar);
			strcat(pngdirectory,"\\KinectScreenshot_RGB%i.png");

			SetDlgItemText(reinterpret_cast <HWND>(m_hWnd), IDC_rgbsaved, _T("Saving "));
			// Save color data
			for(int StartIndex = 0; StartIndex < IntervalThread1; StartIndex++)
			{
				createAlphaMat(mat,StartIndex);

				_snprintf(namepng, sizeof(char) * MAX_PATH, pngdirectory, StartIndex);	
				imwrite(namepng, mat, compression_params);

			}
		}


	}	
	
	while (!thread_handles.empty()) 
	{
		ary.assign(thread_handles.begin(), thread_handles.end());
		DWORD rc = ::WaitForMultipleObjects(ary.size(), &ary[0], FALSE, INFINITE);
		if (rc == WAIT_FAILED)
			{
				// handle a failure case... this is usually something pretty bad
				break;
			} 
		else 
			{
				long idx = (rc - WAIT_OBJECT_0);
				if (idx >= 0 && idx < ary.size())
					{
						// the object at `idx` was signaled, this means that the thread has terminated.
						thread_handles.erase(ary[idx]);
						::CloseHandle(ary[idx]); // necessary with _beginthreadex
					}
			}
	}

	if(checkColor)
	{
		SetDlgItemText(reinterpret_cast <HWND>(m_hWnd), IDC_rgbsaved, _T("Done "));
	}

	return 0;
}



/// <summary>
/// Thread that saves all the data to disk
/// </summary>
unsigned  __stdcall SaveThread2(void * m_hWnd)
{
	if(checkColor)
	{
		if(ThreadsSupported==4)
		{
			if((contClickBmp%2)==1)			
			{
				SetDlgItemText(reinterpret_cast <HWND>(m_hWnd), IDC_rgbsaved, _T("Saving "));
				// Save color data
				for(int StartIndex = IntervalThread1; StartIndex < IntervalThread2; StartIndex++)
				{
					StringCchPrintfW(DataFileName2,_countof(DataFileName2),L"%s\\KinectScreenshot_RGB%d.bmp", FolderName_c, StartIndex);	
					SaveBitmapToFile(reinterpret_cast <BYTE*>(ColorArrayBuffer[StartIndex]), Color_col, Color_row, sizeof(RGBQUAD) * 8, DataFileName2);
				
				}
			}

			if((contClickPng%2)==1)			
			{
				Mat mat2(1080, 1920, CV_8UC4);

				vector<int> compression_params;
				compression_params.push_back(IMWRITE_PNG_COMPRESSION);
				compression_params.push_back(9);

				RGBQUAD* pBuffer;


				char pngdirectory[MAX_PATH]; 
				_bstr_t b(FolderName_png);				
				const char* Foldername_pngchar = b;
				printf("%s", Foldername_pngchar);
				strcpy(pngdirectory,Foldername_pngchar);
				strcat(pngdirectory,"\\KinectScreenshot_RGB%i.png");

				SetDlgItemText(reinterpret_cast <HWND>(m_hWnd), IDC_rgbsaved, _T("Saving "));
				// Save color data
				for(int StartIndex = IntervalThread1; StartIndex < IntervalThread2; StartIndex++)
				{
					createAlphaMat2(mat2,StartIndex);

					_snprintf(namepng, sizeof(char) * MAX_PATH, pngdirectory, StartIndex);	

					imwrite(namepng, mat2, compression_params);

				}
			}
		}
		else
		{
			if((contClickBmp%2)==1)		
			{

				// Save color data
				for(int StartIndex = IntervalThread1; StartIndex < TotalColorFrameCaptured; StartIndex++)
				{
					StringCchPrintfW(DataFileName2,_countof(DataFileName2),L"%s\\KinectScreenshot_RGB%d.bmp", FolderName_c, StartIndex);	
					SaveBitmapToFile(reinterpret_cast <BYTE*>(ColorArrayBuffer[StartIndex]), Color_col, Color_row, sizeof(RGBQUAD) * 8, DataFileName2);
			
					SetDlgItemText(reinterpret_cast <HWND>(m_hWnd), IDC_rgbsaved, _T("Saving "));

				}
			}

			if((contClickPng%2)==1)		
			{
				Mat mat2(1080, 1920, CV_8UC4);

				vector<int> compression_params;
				compression_params.push_back(IMWRITE_PNG_COMPRESSION);
				compression_params.push_back(9);
	

				RGBQUAD* pBuffer;

				char pngdirectory[MAX_PATH]; 
				_bstr_t b(FolderName_png);				
				const char* Foldername_pngchar = b;
				printf("%s", Foldername_pngchar);
				strcpy(pngdirectory,Foldername_pngchar);
				strcat(pngdirectory,"\\KinectScreenshot_RGB%i.png");
		
				// Save color data
				for(int StartIndex = IntervalThread1; StartIndex < TotalColorFrameCaptured; StartIndex++)
				{
					createAlphaMat2(mat2,StartIndex);

					_snprintf(namepng, sizeof(char) * MAX_PATH, pngdirectory, StartIndex);	
					imwrite(namepng, mat2, compression_params);

					SetDlgItemText(reinterpret_cast <HWND>(m_hWnd), IDC_rgbsaved, _T("Saving "));

				}
			}
		}
	}
	return 0;
}


/// <summary>
/// Thread that saves all the data to disk
/// </summary>
unsigned  __stdcall SaveThread3(void * m_hWnd)
{
	if(checkColor)
	{
		if((contClickBmp%2)==1)			
		{
			// Save color data
			for(int StartIndex = IntervalThread2; StartIndex < IntervalThread3; StartIndex++)
				{
					StringCchPrintfW(DataFileName3,_countof(DataFileName3),L"%s\\KinectScreenshot_RGB%d.bmp", FolderName_c, StartIndex);	
					SaveBitmapToFile(reinterpret_cast <BYTE*>(ColorArrayBuffer[StartIndex]), Color_col, Color_row, sizeof(RGBQUAD) * 8, DataFileName3);
				}
		}

		if((contClickPng%2)==1)			
		{
			Mat mat3(1080, 1920, CV_8UC4);

			vector<int> compression_params;
			compression_params.push_back(IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(9);

			RGBQUAD* pBuffer;

			char pngdirectory[MAX_PATH]; 
			_bstr_t b(FolderName_png);				
			const char* Foldername_pngchar = b;
			printf("%s", Foldername_pngchar);
			strcpy(pngdirectory,Foldername_pngchar);
			strcat(pngdirectory,"\\KinectScreenshot_RGB%i.png");

			// Save color data
			for(int StartIndex = IntervalThread2; StartIndex < IntervalThread3; StartIndex++)
				{
					createAlphaMat3(mat3,StartIndex);

					_snprintf(namepng, sizeof(char) * MAX_PATH, pngdirectory, StartIndex);	

					imwrite(namepng, mat3, compression_params);
				}
		}
	}

	return 0;
}



/// <summary>
/// Thread that saves all the data to disk
/// </summary>
unsigned  __stdcall SaveThread4(void * m_hWnd)
{
	if(checkColor)
	{
		if((contClickBmp%2)==1)			
		{
			// Save color data
			for(int StartIndex = IntervalThread3 ; StartIndex < TotalColorFrameCaptured; StartIndex++)
				{
					StringCchPrintfW(DataFileName4,_countof(DataFileName4),L"%s\\KinectScreenshot_RGB%d.bmp", FolderName_c, StartIndex);	
					SaveBitmapToFile(reinterpret_cast <BYTE*>(ColorArrayBuffer[StartIndex]), Color_col, Color_row, sizeof(RGBQUAD) * 8, DataFileName4);

				}
		}

		if((contClickPng%2)==1)			
		{
			Mat mat4(1080, 1920, CV_8UC4);

			vector<int> compression_params;
			compression_params.push_back(IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(9);

			RGBQUAD* pBuffer;

			char pngdirectory[MAX_PATH]; 
			_bstr_t b(FolderName_png);				
			const char* Foldername_pngchar = b;
			printf("%s", Foldername_pngchar);
			strcpy(pngdirectory,Foldername_pngchar);
			strcat(pngdirectory,"\\KinectScreenshot_RGB%i.png");

			// Save color data
			for(int StartIndex = IntervalThread3 ; StartIndex < TotalColorFrameCaptured; StartIndex++)
				{
					createAlphaMat4(mat4,StartIndex);

					_snprintf(namepng, sizeof(char) * MAX_PATH, pngdirectory, StartIndex);	

					imwrite(namepng, mat4, compression_params);

				}
		}
	}

	return 0;
}



/// <summary>
/// Creates all the necessary folders
/// </summary>
void  CCompleteViewer::CreateFolders()
{
	
	StringCchPrintfW(FolderName_d,_countof(FolderName_d),L"%s\\Depth", FolderName);	
					if (GetFileAttributes(FolderName_d) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_d,NULL);
    StringCchPrintfW(FolderName_c,_countof(FolderName_c),L"%s\\Color", FolderName);	
					if (GetFileAttributes(FolderName_c) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_c,NULL);
    StringCchPrintfW(FolderName_b,_countof(FolderName_b),L"%s\\Body", FolderName);	
					if (GetFileAttributes(FolderName_b) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_b,NULL);
    StringCchPrintfW(FolderName_i,_countof(FolderName_i),L"%s\\Infrared", FolderName);	
					if (GetFileAttributes(FolderName_i) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_i,NULL);
    StringCchPrintfW(FolderName_m,_countof(FolderName_m),L"%s\\Mapp", FolderName);	
					if (GetFileAttributes(FolderName_m) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_m,NULL);
    StringCchPrintfW(FolderName_t,_countof(FolderName_t),L"%s\\Time", FolderName);	
					 if (GetFileAttributes(FolderName_t) == INVALID_FILE_ATTRIBUTES) 
						 CreateDirectory(FolderName_t,NULL);
	StringCchPrintfW(FolderName_png,_countof(FolderName_png),L"%s\\ColorPng", FolderName);	
					if (GetFileAttributes(FolderName_png) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_png,NULL);
}

/// <summary>
/// It's executes when the button "Save data" is pushed
/// </summary>
void CCompleteViewer::Save()
{

	ThreadsSupported = std::thread::hardware_concurrency();
	if(ThreadsSupported>ThreadsMax)	
		{
			ThreadsSupported = ThreadsMax;
		}

	EnableWindow(GetDlgItem(m_hWnd, IDC_Start), FALSE);
	EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), FALSE);
	EnableWindow(GetDlgItem(m_hWnd, IDC_Allocates), FALSE);

	// First thread  to save data
	_beginthreadex(NULL, 0, SaveThread, reinterpret_cast <void *>(m_hWnd), 0, NULL);

}



/// <summary>
/// It's executes when the button "Allocates memory" is pushed
/// </summary>
void CCompleteViewer::StartAllocates()
{
	MEMORYSTATUSEX memInfo;
    memInfo.dwLength = sizeof(MEMORYSTATUSEX);
    GlobalMemoryStatusEx(&memInfo);
	DWORDLONG physMemAvail = memInfo.ullAvailPhys;

	// Check and create the folder in C(default) or to the path passed 
	PathFileName = (LPTSTR)GlobalAlloc(GPTR, MAX_PATH);
		GetDlgItemText(m_hWnd, IDC_path, PathFileName, MAX_PATH);
	if (PathFileName == NULL || PathFileName[0] == 0)
		StringCchPrintfW(FolderName,_countof(FolderName),L"C:\\Data_v2");
	else
	{
		//Save in FolderName the complete path
		StringCchPrintfW(FolderName,_countof(FolderName),L"%s\\Data_v2",PathFileName);
		//Find single name of subfolders
		int idx_char = 0;
		while(FolderName[idx_char] !='\0')
		{
			//not create the first drive (default:'C')
			FolderNameTemp[idx_char] = FolderName[idx_char];
			if(FolderName[idx_char] == '\\')
			{
				idx_char++;
				break;
			}
			idx_char++;
		}
		while(FolderName[idx_char] !='\0')
		{
			FolderNameTemp[idx_char] = FolderName[idx_char];
			if(FolderName[idx_char] == '\\')
			{
				//create subfolders
				if (GetFileAttributes(FolderNameTemp) == INVALID_FILE_ATTRIBUTES) 
				CreateDirectory(FolderNameTemp,NULL);
			}
			idx_char++;
		}
	}
	// Create a Directory in a specified local disk name// 
	if (GetFileAttributes(FolderName) == INVALID_FILE_ATTRIBUTES) 
		CreateDirectory(FolderName,NULL);

    // Create folders if don't exist
	CreateFolders();

	// Check if there is a maximum number of framerate RGB
	NumFps = (LPTSTR)GlobalAlloc(GPTR, sizeof(LPTSTR));
	if (GetDlgItemText(m_hWnd, IDC_fpsRGB, NumFps, sizeof(LPTSTR)))
	{
		NumFpsCapture = _wtoi(NumFps); 
		// Checks that the value passed from GUI is >=1 and <=30                           
		if(NumFpsCapture>MAX_FPS)
		{ 
			NumFpsCapture = MAX_FPS;
			SetDlgItemText(m_hWnd, IDC_fpsRGB, L"30");
		}
		else if(NumFpsCapture<1)
		{
			NumFpsCapture = 1;
			SetDlgItemText(m_hWnd, IDC_fpsRGB, L"1");
		}
		recfps = std::floor(30/NumFpsCapture);
		NumFramesAllocateRGB = NumFpsCapture*NumAcquisitionSec;
	}
	
	// Check if there is a maximum number of framerate Depth
	NumFpsDepth = (LPTSTR)GlobalAlloc(GPTR, sizeof(LPTSTR));
	if (GetDlgItemText(m_hWnd, IDC_fpsDepth, NumFpsDepth, sizeof(LPTSTR)))
	{
		NumFpsCaptureDepth = _wtoi(NumFpsDepth);
		// Checks that the value passed from GUI is >=1 and <=30                       
		if(NumFpsCaptureDepth>MAX_FPS)
		{ 
			NumFpsCaptureDepth = MAX_FPS;
			SetDlgItemText(m_hWnd, IDC_fpsDepth, L"30");
		}
		else if(NumFpsCaptureDepth<1)
		{
			NumFpsCaptureDepth = 1;
			SetDlgItemText(m_hWnd, IDC_fpsDepth, L"1");
		}
		recfpsDepth = std::floor(30/NumFpsCaptureDepth);
		NumFramesAllocateDepth = NumFpsCaptureDepth*NumAcquisitionSec;
	}

	// Check if there is a maximum number of framerate IR
	NumFpsIR = (LPTSTR)GlobalAlloc(GPTR, MAX_FPS);
	if (GetDlgItemText(m_hWnd, IDC_fpsIR, NumFpsIR, MAX_FPS))
	{
		NumFpsCaptureIR = _wtoi(NumFpsIR); 
		// Checks that the value passed from GUI is >=1 and <=30                       
		if(NumFpsCaptureIR>MAX_FPS)
		{ 
			NumFpsCaptureIR = MAX_FPS;
			SetDlgItemText(m_hWnd, IDC_fpsIR, L"30");
		}
		else if(NumFpsCaptureIR<1)
		{
			NumFpsCaptureIR = 1;
			SetDlgItemText(m_hWnd, IDC_fpsIR, L"1");
		}
		recfpsIR = std::floor(30/NumFpsCaptureIR);
		NumFramesAllocateIR = NumFpsCaptureIR*NumAcquisitionSec;
	}

	// Check if there is a maximum number of framerate Skeleton
	NumFpsSkeleton = (LPTSTR)GlobalAlloc(GPTR, MAX_FPS);
	if (GetDlgItemText(m_hWnd, IDC_fpsSkeleton, NumFpsSkeleton, MAX_FPS))
	{
		NumFpsCaptureSkeleton = _wtoi(NumFpsSkeleton); 
		// Checks that the value passed from GUI is >=1 and <=30                            
		if(NumFpsCaptureSkeleton>MAX_FPS)
		{ 
			NumFpsCaptureSkeleton = MAX_FPS;
			SetDlgItemText(m_hWnd, IDC_fpsSkeleton, L"30");
		}
		else if(NumFpsCaptureSkeleton<1)
		{
			NumFpsCaptureSkeleton = 1;
			SetDlgItemText(m_hWnd, IDC_fpsSkeleton, L"1");
		}
		recfpsSkeleton = std::floor(30/NumFpsCaptureSkeleton);
		NumFramesAllocateSkel = NumFpsCaptureSkeleton*NumAcquisitionSec;
	}

	
	// Check if there is equality of mapping     
	NumFpsMapp = (LPTSTR)GlobalAlloc(GPTR, MAX_FPS);
	if (GetDlgItemText(m_hWnd, IDC_fpsRGB, NumFps, MAX_FPS) && GetDlgItemText(m_hWnd, IDC_fpsDepth, NumFpsDepth, MAX_FPS))
	{
		if (NumFpsCapture==NumFpsCaptureDepth)
		{
			NumFpsCaptureMapp = _wtoi(NumFps);
			recfpsMapp = std::floor(30/NumFpsCaptureMapp);
		}
		NumFramesAllocateMapp = NumFpsCapture*NumAcquisitionSec;;
	}


	if(NumFpsCaptureDepth != NumFpsCapture)
		{
			checkMapp = 0;
		}

	//int sizergb = checkColor*_wtoi(NumFps)*8*1024*1024; //if png chosen, data passed in memory are bmp and saved in png format		//notab usare check
	int sizergb = checkColor*_wtoi(NumFps)*4*Color_row*Color_col;
	int sizeir = checkIR*_wtoi(NumFpsIR)*900*1024;
	int sizemapp = checkMapp*_wtoi(NumFps)*900*1024;
	//int sizedepth = checkDepth*_wtoi(NumFpsDepth)*450*1024;
	int sizedepth = checkDepth*_wtoi(NumFpsDepth)*depth_row*depth_col*2;
	int sizeskel = checkSkeleton*_wtoi(NumFpsSkeleton)*30*1024;


	secondsAvail = (RAM_PERC*physMemAvail)/ (sizedepth+sizeir+sizemapp+sizergb+sizeskel);

	// Check if there is a maximum of acquisition time
	NumAcquisitionTime = (LPTSTR)GlobalAlloc(GPTR, sizeof(LPTSTR));
	if (GetDlgItemText(m_hWnd, IDC_AcquisitionTime, NumAcquisitionTime, sizeof(LPTSTR)))
	{
		int minlimitsec;
		NumAcquisitionSec = _wtoi(NumAcquisitionTime);

		if(secondsAvail>SOFTWARELIMITSEC)
			{
				minlimitsec = SOFTWARELIMITSEC;
			}
		else 
			{
				minlimitsec = secondsAvail;
			}

		//if(NumAcquisitionSec>secondsAvail)	
		if(NumAcquisitionSec>minlimitsec)		
			{
				NumAcquisitionSec = minlimitsec;
			}
		else if(NumAcquisitionSec<1)
			{
				NumAcquisitionSec = 1;
			}
		else
			{
				NumAcquisitionSec = _wtoi(NumAcquisitionTime);
			}
				
			NumFramesAllocateRGB = NumAcquisitionSec*NumFpsCapture;
			NumFramesAllocateSkel = NumAcquisitionSec*NumFpsCaptureSkeleton;
			NumFramesAllocateMapp = NumAcquisitionSec*NumFpsCaptureMapp;
			NumFramesAllocateIR = NumAcquisitionSec*NumFpsCaptureIR;
			NumFramesAllocateDepth = NumAcquisitionSec*NumFpsCaptureDepth;	
			
			wsprintfW(bufferAvRam, L"%d", NumAcquisitionSec);

			SetDlgItemText(m_hWnd, IDC_AcquisitionTime, bufferAvRam);
	}

	Allocate();
	ClickAllocates = 1;

	EnableWindow(GetDlgItem(m_hWnd, IDC_Start), TRUE);
	EnableWindow(GetDlgItem(m_hWnd, IDC_Save), FALSE);   
	EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK2), FALSE);	
	EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK3), FALSE);	
	EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK4), FALSE);	
	EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK5), FALSE);	
	EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK6), FALSE);	
	EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK7), FALSE);	
	EnableWindow(GetDlgItem(m_hWnd, IDC_numframes), FALSE); 
	EnableWindow(GetDlgItem(m_hWnd, IDC_Allocates), FALSE); 
	EnableWindow(GetDlgItem(m_hWnd, IDC_fpsRGB), FALSE);	
	EnableWindow(GetDlgItem(m_hWnd, IDC_fpsDepth), FALSE);	
	EnableWindow(GetDlgItem(m_hWnd, IDC_fpsIR), FALSE);	
	EnableWindow(GetDlgItem(m_hWnd, IDC_fpsSkeleton), FALSE);
	EnableWindow(GetDlgItem(m_hWnd, IDC_AcquisitionTime), FALSE);
}

/// <summary>
/// Reports when the program finishes to capture the data
/// </summary>
bool CCompleteViewer::EndCapture()
{
	if(checkColor && checkIR && checkDepth && checkSkeleton && checkMapp)
	{
		if(TotalColorFrameCaptured == NumFramesAllocateRGB && TotalIRFrameCaptured == NumFramesAllocateIR && TotalDepthFrameCaptured == NumFramesAllocateDepth && 
			TotalSkeletonFrameCaptured == NumFramesAllocateSkel && TotalMappFrameCaptured == NumFramesAllocateMapp)
		{
			return true;
		}
	}

	if(checkColor && checkIR && checkDepth && checkSkeleton && !checkMapp)
	{
		if(TotalColorFrameCaptured == NumFramesAllocateRGB && TotalIRFrameCaptured == NumFramesAllocateIR && TotalDepthFrameCaptured == NumFramesAllocateDepth && 
			TotalSkeletonFrameCaptured == NumFramesAllocateSkel)
		{
			return true;
		}
	}

	if(checkColor && checkIR && checkDepth && checkMapp && !checkSkeleton)
	{
		if(TotalColorFrameCaptured == NumFramesAllocateRGB && TotalIRFrameCaptured == NumFramesAllocateIR && TotalDepthFrameCaptured == NumFramesAllocateDepth && TotalMappFrameCaptured == NumFramesAllocateMapp)
		{
			return true;
		}
	}

	if(checkColor && checkIR && checkDepth && !checkSkeleton && !checkMapp)
	{
		if(TotalColorFrameCaptured == NumFramesAllocateRGB && TotalIRFrameCaptured == NumFramesAllocateIR && TotalDepthFrameCaptured == NumFramesAllocateDepth)
		{
			return true;
		}
	}

	if(checkColor && checkIR && checkSkeleton && !checkDepth && !checkMapp)
	{
		if(TotalColorFrameCaptured == NumFramesAllocateRGB && TotalIRFrameCaptured == NumFramesAllocateIR && TotalSkeletonFrameCaptured == NumFramesAllocateSkel)
		{
			return true;
		}
	}

	if(checkDepth && checkIR && checkSkeleton && !checkColor && !checkMapp)
	{
		if(TotalDepthFrameCaptured == NumFramesAllocateDepth && TotalIRFrameCaptured == NumFramesAllocateIR && TotalSkeletonFrameCaptured == NumFramesAllocateSkel)
		{
			return true;
		}
	}

	if(checkDepth && checkColor && checkSkeleton && checkMapp && !checkIR)
	{
		if(TotalDepthFrameCaptured == NumFramesAllocateDepth && TotalColorFrameCaptured == NumFramesAllocateRGB && TotalSkeletonFrameCaptured == NumFramesAllocateSkel && TotalMappFrameCaptured == NumFramesAllocateMapp)
		{
			return true;
		}
	}

	if(checkDepth && checkColor && checkSkeleton && !checkIR && !checkMapp)
	{
		if(TotalDepthFrameCaptured == NumFramesAllocateDepth && TotalColorFrameCaptured == NumFramesAllocateRGB && TotalSkeletonFrameCaptured == NumFramesAllocateSkel)
		{
			return true;
		}
	}

	if(checkDepth && checkColor && checkIR && checkMapp && !checkSkeleton)
	{
		if(TotalDepthFrameCaptured == NumFramesAllocateDepth && TotalColorFrameCaptured == NumFramesAllocateRGB && TotalIRFrameCaptured == NumFramesAllocateIR && TotalMappFrameCaptured == NumFramesAllocateMapp)
		{
			return true;
		}
	}

	if(checkDepth && checkColor && checkIR && !checkSkeleton && !checkMapp)
	{
		if(TotalDepthFrameCaptured == NumFramesAllocateDepth && TotalColorFrameCaptured == NumFramesAllocateRGB && TotalIRFrameCaptured == NumFramesAllocateIR)
		{
			return true;
		}
	}

	if(checkDepth && checkSkeleton && !checkColor && !checkIR && !checkMapp)
	{
		if(TotalDepthFrameCaptured == NumFramesAllocateDepth && TotalSkeletonFrameCaptured == NumFramesAllocateSkel)
		{
			return true;
		}
	}

	if(checkIR && checkColor && !checkDepth && !checkSkeleton && !checkMapp)
	{
		if(TotalIRFrameCaptured == NumFramesAllocateIR && TotalColorFrameCaptured == NumFramesAllocateRGB)
		{
			return true;
		}
	}

	if(checkDepth && checkColor && checkMapp && !checkIR && !checkSkeleton)
	{
		if(TotalDepthFrameCaptured == NumFramesAllocateDepth && TotalColorFrameCaptured == NumFramesAllocateRGB && TotalMappFrameCaptured == NumFramesAllocateMapp)
		{
			return true;
		}
	}

	if(checkDepth && checkColor && !checkIR && !checkSkeleton && !checkMapp)
	{
		if(TotalDepthFrameCaptured == NumFramesAllocateDepth && TotalColorFrameCaptured == NumFramesAllocateRGB)
		{
			return true;
		}
	}

	if(checkDepth && checkIR && !checkColor && !checkMapp && !checkSkeleton)
	{
		if(TotalDepthFrameCaptured == NumFramesAllocateDepth && TotalIRFrameCaptured == NumFramesAllocateIR)
		{
			return true;
		}
	}

	if(checkIR && checkSkeleton && !checkDepth && !checkColor && !checkMapp)
	{
		if(TotalIRFrameCaptured == NumFramesAllocateIR && TotalSkeletonFrameCaptured == NumFramesAllocateSkel)
		{
			return true;
		}
	}

	if(checkColor && checkSkeleton && !checkIR && !checkDepth && !checkMapp)
	{
		if(TotalColorFrameCaptured == NumFramesAllocateRGB && TotalSkeletonFrameCaptured == NumFramesAllocateSkel)
		{
			return true;
		}
	}

	if(checkColor && !checkSkeleton && !checkIR && !checkDepth && !checkMapp)
	{
		if(TotalColorFrameCaptured == NumFramesAllocateRGB)
		{
			return true;
		}
	}

	if(checkIR && !checkColor && !checkSkeleton && !checkDepth && !checkMapp)
	{
		if(TotalIRFrameCaptured == NumFramesAllocateIR)
		{
			return true;
		}
	}

	if(checkSkeleton && !checkIR && !checkColor && !checkDepth && !checkMapp)
	{
		if(TotalSkeletonFrameCaptured == NumFramesAllocateSkel)
		{
			return true;
		}
	}

	if(checkDepth && !checkSkeleton && !checkIR && !checkColor && !checkMapp)
	{
		if(TotalDepthFrameCaptured == NumFramesAllocateDepth)
		{
			return true;
		}
	}

	return false;
}

/// <summary>
/// It's executes when the button "Start capture" is pushed
/// </summary>
void CCompleteViewer::StartCapture()
{
	if(checkColor && checkIR && checkDepth && checkSkeleton && !checkMapp)
	{
		CaptureSkeletonFlag = 1;
		CaptureDepthFlag = 1;
		CaptureColorFlag = 1;
		CaptureIRFlag = 1;	
		
		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkColor && checkIR && checkDepth && checkSkeleton && checkMapp)
	{
		CaptureSkeletonFlag = 1;
		CaptureDepthFlag = 1;
		CaptureColorFlag = 1;
		CaptureIRFlag = 1;
		CaptureMappFlag =1;
		
		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}
	
	if(checkColor && checkIR && checkDepth && !checkSkeleton && !checkMapp)
	{
		CaptureColorFlag = 1;
		CaptureIRFlag = 1;
		CaptureDepthFlag = 1;	
	
        EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkColor && checkIR && checkDepth && checkMapp && !checkSkeleton)
	{
		CaptureColorFlag = 1;
		CaptureIRFlag = 1;
		CaptureDepthFlag = 1;
		CaptureMappFlag = 1;
		
	   EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkColor && checkIR && checkSkeleton && !checkDepth && !checkMapp)
	{
		CaptureColorFlag = 1;
		CaptureIRFlag = 1;
		CaptureSkeletonFlag = 1;
		
	    EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkDepth && checkIR && checkSkeleton && !checkColor && !checkMapp)
	{
		CaptureDepthFlag = 1;
		CaptureIRFlag = 1;
		CaptureSkeletonFlag = 1;
	
	    EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkDepth && checkColor && checkSkeleton && !checkIR && !checkMapp)
	{
		CaptureDepthFlag = 1;
		CaptureColorFlag = 1;
		CaptureSkeletonFlag = 1;
		
		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkDepth && checkColor && checkSkeleton && checkMapp && !checkIR)
	{
		CaptureDepthFlag = 1;
		CaptureColorFlag = 1;
		CaptureSkeletonFlag = 1;
		CaptureMappFlag = 1;
		
		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkDepth && checkColor && checkIR && !checkSkeleton && !checkMapp)
	{
		CaptureDepthFlag = 1;
		CaptureColorFlag = 1;
		CaptureIRFlag = 1;
		
		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkDepth && checkColor && checkIR && checkMapp && !checkSkeleton)
	{
		CaptureDepthFlag = 1;
		CaptureColorFlag = 1;
		CaptureIRFlag = 1;
		CaptureMappFlag = 1;
		
	    EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkDepth &&  checkSkeleton && !checkColor && !checkIR && !checkMapp)
	{
		CaptureDepthFlag = 1;
		CaptureSkeletonFlag = 1;
				
		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkIR && checkColor && !checkDepth &&  !checkSkeleton && !checkMapp)
	{	
		CaptureIRFlag = 1;
		CaptureColorFlag = 1;
		
		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkDepth && checkColor && !checkIR && !checkSkeleton && !checkMapp)
	{		
		CaptureDepthFlag = 1;
		CaptureColorFlag = 1;
	
	    EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkDepth && checkColor && checkMapp && !checkIR && !checkSkeleton)
	{	
		CaptureDepthFlag = 1;
		CaptureColorFlag = 1;
		CaptureMappFlag = 1;
	
	    EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkDepth && checkIR && !checkColor && !checkMapp && !checkSkeleton)
	{
		CaptureDepthFlag = 1;
		CaptureIRFlag = 1;
		
		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkIR && checkSkeleton && !checkDepth && !checkColor && !checkMapp)
	{
		CaptureIRFlag = 1;
		CaptureSkeletonFlag = 1;
		
		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkColor && checkSkeleton && !checkIR && !checkDepth && !checkMapp)
	{
		CaptureColorFlag = 1;
		CaptureSkeletonFlag = 1;

		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkColor && !checkSkeleton && !checkIR && !checkDepth && !checkMapp)
	{	
		CaptureColorFlag = 1;
			
		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkIR && !checkColor && !checkSkeleton && !checkDepth && !checkMapp)
	{
		CaptureIRFlag = 1;
			
		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkSkeleton && !checkIR && !checkColor && !checkDepth && !checkMapp)
	{
		CaptureSkeletonFlag = 1;
	
		EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}

	if(checkDepth && !checkSkeleton && !checkIR && !checkColor && !checkMapp)
	{
		CaptureDepthFlag = 1;
		
	    EnableWindow(GetDlgItem(m_hWnd, IDC_Stop), TRUE);
	}
}


/// <summary>
/// It's executes when the button "Stop capture" is pushed
/// </summary>
void CCompleteViewer::StopCapture()
{
	EnableWindow(GetDlgItem(m_hWnd, IDC_Save), TRUE);
	EnableWindow(GetDlgItem(m_hWnd, IDC_Start), TRUE);

	if(CaptureColorFlag)
	{
		CaptureColorFlag = 0;
	}

	if(CaptureIRFlag)
	{
		CaptureIRFlag = 0;
	}

	if(CaptureDepthFlag)
	{
		CaptureDepthFlag = 0;
	}

	if(CaptureSkeletonFlag)
	{
		CaptureSkeletonFlag = 0;
	}

	if(CaptureMappFlag)
	{
		CaptureMappFlag = 0;
	}
}  
