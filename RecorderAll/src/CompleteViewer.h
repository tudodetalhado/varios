//------------------------------------------------------------------------------
// <copyright file="CompleteViewer.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "ImageRenderer.h"

#define WM_USER_UPDATE_DEPTH			WM_USER
#define WM_USER_UPDATE_SKELETON			WM_USER+1
#define WM_USER_UPDATE_RGB				WM_USER+2
#define WM_USER_UPDATE_IR               WM_USER+3  
#define WM_USER_UPDATE_MAPP             WM_USER+4

class CCompleteViewer
{

public:
	/// <summary>
	/// Constructor
	/// </summary>
	CCompleteViewer();

	/// <summary>
	/// Destructor
	/// </summary>
	~CCompleteViewer();

	/// <summary>
	/// Handles window messages, passes most to the class instance to handle
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Handle windows messages for a class instance
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Creates the main window and begins processing
	/// </summary>
	/// <param name="hInstance"></param>
	/// <param name="nCmdShow"></param>
	int                     Run(HINSTANCE hInstance, int nCmdShow);

private:
	HWND                    m_hWnd;
	INT64                   m_nStartTime;
	INT64                   m_nLastCounter;
	double                  m_fFreq;
	DWORD                   m_nNextStatusTime;
	DWORD                   m_nFramesSinceUpdate;

	// Current Kinect
	IKinectSensor*          m_pKinectSensor;
	ColorSpacePoint*        m_pColorCoordinates;
	ICoordinateMapper*      m_pCoordinateMapper;

	// Frame reader
	IMultiSourceFrameReader*m_pMultiSourceFrameReader;

	// Direct2D
	ImageRenderer*          m_pDrawDepth;
	ImageRenderer*          m_pDrawDepth2;
	ImageRenderer*          m_pDrawColor;
	RGBQUAD*                m_pColorRGBX;
	ID2D1Factory*           m_pD2DFactory;
	RGBQUAD*                m_pDepthRGBX;

	// Body/hand drawing
	ID2D1HwndRenderTarget*  m_pRenderTarget;
	ID2D1SolidColorBrush*   m_pBrushJointTracked;
	ID2D1SolidColorBrush*   m_pBrushJointInferred;
	ID2D1SolidColorBrush*   m_pBrushBoneTracked;
	ID2D1SolidColorBrush*   m_pBrushBoneInferred;
	ID2D1SolidColorBrush*   m_pBrushHandClosed;
	ID2D1SolidColorBrush*   m_pBrushHandOpen;
	ID2D1SolidColorBrush*   m_pBrushHandLasso;

	// Direct2D
	ImageRenderer*          m_pDrawCoordinateMapping;
	ImageRenderer*          m_pDrawInfrared;
	RGBQUAD*                m_pInfraredRGBX;
	RGBQUAD*                m_pOutputRGBX; 
	RGBQUAD*                m_pBackgroundRGBX; 

	//PNG
	//YsRawPngEncoder          m_PNGEncoder;

	/// <summary>
	/// Main processing function
	/// </summary>
	void                    Update();

	/// <summary>
	/// Initializes the default Kinect sensor
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                 InitializeDefaultSensor();

	/// <summary>
	/// Handle new depth data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="pBuffer">pointer to frame data</param>
	/// <param name="nWidth">width (in pixels) of input image data</param>
	/// <param name="nHeight">height (in pixels) of input image data</param>
	/// <param name="nMinDepth">minimum reliable depth</param>
	/// <param name="nMaxDepth">maximum reliable depth</param>
	/// </summary>
	void                    ProcessDepth(INT64 nTime, UINT16* pBuffer, int nHeight, int nWidth, USHORT nMinDepth, USHORT nMaxDepth);

	/// <summary>
	/// Handle new color data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="pBuffer">pointer to frame data</param>
	/// <param name="nWidth">width (in pixels) of input image data</param>
	/// <param name="nHeight">height (in pixels) of input image data</param>
	/// </summary>
	void                    ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight);

	/// <summary>
	/// Create and save the mapping matrix into the buffer
	/// <param name="pDepthBuffer">pointer to frame data</param>
	/// <param name="nDepthWidth">width (in pixels) of input image data</param>
	/// <param name="nDepthHeight">height (in pixels) of input image data</param>
	/// </summary>
	void                    ProcessMapp(UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight);


	/// <summary>
	/// Handle new infrared data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="pBuffer">pointer to frame data</param>
	/// <param name="nWidth">width (in pixels) of input image data</param>
	/// <param name="nHeight">height (in pixels) of input image data</param>
	/// </summary>
	void                    ProcessInfrared(INT64 nTime, const UINT16* pBuffer, int nHeight, int nWidth);

	/// <summary>
	/// Handle new body data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="nBodyCount">body data count</param>
	/// <param name="ppBodies">body data in frame</param>
	/// </summary>
	void                    ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies);

	/// <summary>
	/// Ensure necessary Direct2d resources are created
	/// </summary>
	/// <returns>S_OK if successful, otherwise an error code</returns>
	HRESULT EnsureDirect2DResources();

	/// <summary>
	/// Dispose Direct2d resources 
	/// </summary>
	void DiscardDirect2DResources();

	/// <summary>
	/// Converts a body point to screen space and save body in a buffer
	/// </summary>
	/// <param name="bodyPoint">body point to tranform</param>
	/// <param name="jointState">the joint state</param>
	/// <param name="width">width (in pixels) of output buffer</param>
	/// <param name="height">height (in pixels) of output buffer</param>
	/// <param name="i">index of body captured</param>
	/// <param name="j">index of joint captured</param>
	/// <returns>point in screen-space</returns>
	D2D1_POINT_2F           BodyToScreen(const CameraSpacePoint& bodyPoint, TrackingState jointState, int width, int height , int i, int j);


	/// <summary>
	/// Converts a body point to screen space and save body in a buffer
	/// </summary>
	/// <param name="bodyPoint">body point to tranform</param>
	/// <param name="jointState">the joint state</param>
	/// <param name="width">width (in pixels) of output buffer</param>
	/// <param name="height">height (in pixels) of output buffer</param>
	/// <returns>point in screen-space</returns>
	D2D1_POINT_2F           BodyToScreenPlot(const CameraSpacePoint& bodyPoint, TrackingState jointState, int width, int height);

	/// <summary>
	/// Draws a body 
	/// </summary>
	/// <param name="pJoints">joint data</param>
	/// <param name="pJointPoints">joint positions converted to screen space</param>
	void                    DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints);

	/// <summary>
	/// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
	/// </summary>
	/// <param name="handState">state of the hand</param>
	/// <param name="handPosition">position of the hand</param>
	void                    DrawHand(HandState handState, const D2D1_POINT_2F& handPosition);

	/// <summary>
	/// Draws one bone of a body (joint to joint)
	/// </summary>
	/// <param name="pJoints">joint data</param>
	/// <param name="pJointPoints">joint positions converted to screen space</param>
	/// <param name="pJointPoints">joint positions converted to screen space</param>
	/// <param name="joint0">one joint of the bone to draw</param>
	/// <param name="joint1">other joint of the bone to draw</param>
	void                    DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1);

	/// <summary>
	/// Set the status bar message
	/// </summary>
	/// <param name="szMessage">message to display</param>
	/// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
	/// <param name="bForce">force status update</param>
	bool                    SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);



	/// <summary>
	/// Creates all the necessary folders
	/// </summary>
	void                    CreateFolders();

	/// <summary>
	/// Allocates the arrays we need
	/// </summary>
	void                    Allocate();

	/// <summary>
	/// Reports when the program finishes to capture the data
	/// </summary>
	bool                    EndCapture();

	/// <summary>
	/// It's executes when the button "Save data" is pushed
	/// </summary>
	void                    Save();

	/// <summary>
	/// It's executes when the button "Allocates memory" is pushed
	/// </summary>
	void                    StartAllocates();

	/// <summary>
	/// It's executes when the button "Start capture packets" is pushed
	/// </summary>
	void                    StartCapture();

	/// <summary>
	/// It's executes when the button "Stop capture" is pushed
	/// </summary>
	void                    StopCapture();
};

