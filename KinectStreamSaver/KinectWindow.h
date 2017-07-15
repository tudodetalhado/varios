//------------------------------------------------------------------------------
// <copyright file="KinectWindow.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include <vector>
#include <NuiApi.h>
#include "NuiDepthStream.h"
#include "NuiColorStream.h"
#include "NuiSkeletonStream.h"
#include "NuiAudioStream.h"
#include "NuiAccelerometerStream.h"
#include "NuiTiltAngleViewer.h"
#include "KinectSettings.h"

#include <fstream>
#include "stdAfx.h" 
#include "Resource.h"
#include "SaverViewer.h"
#include "StreamSaver.h"


class KinectWindow : public NuiViewer
{
public:

	/// <summary>
	/// Constructor
	/// </summary>
	/// <param name="hInstance">Handle to the application instance</param>
	/// <param name="hWndParent">Handle to main console window</param>
	/// <param name="pNuiSensor">Pointer to Nui sensor instance</param>
	KinectWindow(HINSTANCE hInstance, HWND hWndParent, INuiSensor* pNuiSensor);

	/// <summary>
	/// Destructor. Kinect window object is deleted in its own thread. Can not be deleted in other place explicitly
	/// </summary>
	~KinectWindow();

public:
	/// <summary>
	/// Start a new thread and Kinect window runs in it
	/// </summary>
	/// <returns>Handle to thread</returns>
	HANDLE StartWindow();

	/// <summary>
	/// Explicitly called by main window to close Kinect window and stop the thread on exit of main window
	/// </summary>
	void NotifyOfExit();

	/// <summary>
	/// Returns the handle to the thread of the Kinect window
	/// </summary>
	/// <returns>The thread handle</returns>
	HANDLE GetThreadHandle() const;

private:
	/// <summary>
	/// Initialize common control.
	/// </summary>
	/// <returns>Indicates success or failure</returns>
	static bool InitializeCommonControl();

	/// <summary>
	/// The thread procedure in which the window runs
	/// </summary>
	/// <param name="pThis">The pointer to Kinect window instance</param>
	/// <returns>The result from message loop</returns>
	static DWORD WINAPI ThreadProc(KinectWindow* pThis);

	/// <summary>
	/// Thread to save the data streams
	/// </summary>
	/// <param name="pParam">instance pointer</param>
	/// <returns>always 0</returns>
	static DWORD WINAPI     SaveDepthThread( LPVOID pParam );

	/// <summary>
	/// Thread to save the data streams
	/// </summary>
	/// <param name="pParam">instance pointer</param>
	/// <returns>always 0</returns>
	static DWORD WINAPI     SaveColorThread( LPVOID pParam );

	/// <summary>
	/// Thread to save the data streams
	/// </summary>
	/// <param name="pParam">instance pointer</param>
	/// <returns>always 0</returns>
	static DWORD WINAPI     SaveSkelThread( LPVOID pParam );

	/// <summary>
	/// Thread to save the data streams
	/// </summary>
	/// <param name="pParam">instance pointer</param>
	/// <returns>always 0</returns>
	static DWORD WINAPI     SaveThread( LPVOID pParam );

	/// <summary>
	/// Thread to save the data streams
	/// </summary>
	/// <returns>always 0</returns>
	DWORD WINAPI            SaveDepthThread( );	

	/// <summary>
	/// Thread to save the data streams
	/// </summary>
	/// <returns>always 0</returns>
	DWORD WINAPI            SaveColorThread( );	

	/// <summary>
	/// Thread to save the data streams
	/// </summary>
	/// <returns>always 0</returns>
	DWORD WINAPI            SaveSkelThread( );	

	/// <summary>
	/// Thread to save the data streams
	/// </summary>
	/// <returns>always 0</returns>
	DWORD WINAPI            SaveThread( );	

	/// <summary>
	/// Dispatch the message to the handler function
	/// </summary>
	/// <param name="hWnd">The handle to the window which receives the message</param>
	/// <param name="uMsg">The message identifier</param>
	/// <param name="wParam">The additional message information</param>
	/// <param name="lParam">The additional message information</param>
	/// <returns>If the message has been processed by handler function, TRUE is returned. Otherwise FALSE is returned and the message is handled by default dialog procedure</returns>
	virtual LRESULT DialogProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Returns ID of the dialog
	/// </summary>
	/// <returns>ID of dialog</returns>
	virtual UINT GetDlgId();

	/// <summary>
	/// Initialization
	/// </summary>
	/// <returns>Indicate success or failure</returns>
	bool Initialize();

	/// <summary>
	/// Create Kinect window and its sub views
	/// </summary>
	/// <returns>Indicates success or failure</returns>
	bool CreateWindows();

	/// <summary>
	/// Create tab control
	/// </summary>
	/// <returns>Indicates success or failure</returns>
	bool CreateTabControl();

	/// <summary>
	/// Insert a tab item to tab control
	/// </summary>
	/// <param name="title">Text displayed on tab item</param>
	/// <param name="index">Index of tab item</param>
	void InsertTabItem(LPWSTR title, int index);

	/// <summary>
	/// Display Kinect window and its sub views.
	/// </summary>
	void ShowWindows();

	/// <summary>
	/// Bring up Kinect window on top of main console window
	/// </summary>
	void BringUpWindow();

	/// <summary>
	/// Release all resources
	/// </summary>
	void CleanUp();

	/// <summary>
	/// Start all streams and timer
	/// </sumamry>
	void StartStreams();

	/// <summary>
	/// Start the waitable timer to trigger process of timed streams
	/// </summary>
	void StartTimer();

	/// <summary>
	/// Process color, depth and skeleton streams
	/// </summary>
	void UpdateStreams();

	/// <summary>
	/// Process audio, accelerometer and tilt angle streams
	/// </summary>
	void UpdateTimedStreams();

	/// <summary>
	/// Create camera setting viewers
	/// </summary>
	bool CreateCameraSettingViews();

	/// <summary>
	/// Initialize menu control on Kinect window. Set initial check status for menu items
	/// </summary>
	void InitializeMenu();

	/// <summary>
	/// Update menu item status
	/// </summary>
	/// <param name="id">Identifier of menu item</param>
	/// <param name="checked">Check status of menu item</param>
	/// <returns>Indicates success or failure<returns>
	bool ProcessMenuItem(UINT id, bool& checked);

	/// <summary>
	/// Enable or disable popup menu
	/// </summary>
	/// <param name="hMenu">Handle to window menu</param>
	/// <param name="subMenuPosition">Position of sub menu which owns the pop up menu item</param>
	/// <param name="popupMenuPosition">Position of popup menu to enable or disable</param>
	/// <param name="enable">True to enable and false to disable the menu</param>
	/// <returns>Indicates success or failure</returns>
	bool EnablePopupMenuItem(HMENU hMenu, UINT subMenuPosition, UINT popupMenuPosition, bool enable);

	/// <summary>
	/// Invert the check status of menu item
	/// </summary>
	/// <param name="hMenu">Handle to window menu</param>
	/// <param name="id">ID of menu item to check or uncheck</param>
	/// <param name="previouslyChecked">Previous check status of menu item</param>
	/// <returns>Indicates success or failure</returns>
	bool InvertCheckMenuItem(HMENU hMenu, UINT id, bool previouslyChecked);

	/// <summary>
	/// Check radio menu item
	/// </summary>
	/// <param name="id">ID of menu item to check<param>
	/// <param name="start">Start ID of grouped radio items</param>
	/// <param name="end">End ID of grouped radio items</param>
	/// <param name="hMenu">Handle to menu</param>
	/// <returns>Indicate success or failure</returns>
	bool CheckRadioItem(UINT id, UINT start, UINT end, HMENU hMenu);

	/// <summary>
	/// Retrieve check status of menu item
	/// </summary>
	/// <param name="hMenu">Handle to menu control</param>
	/// <param name="id">Identifier of menu item</param>
	/// <param name="checked">Retrieve the check status of menu item</param>
	/// <returns>Indicates success or failure</returns>
	bool GetMenuItemCheckStatus(HMENU hMenu, UINT id, bool& checked);

	/// <summary>
	/// Calculates new positions and sizes for sub windows when Kinect window is resized
	/// </summary>
	/// <param name="priRect">New position and size for primary display viewer</param>
	/// <param name="secRect">New position and size for secondary display viewer</param>
	/// <param name="tabRect">New position and size for tab control</param>
	/// <param name="tabbedRect">New position and size for tabbed viewers</param>
	/// <returns>Indicates success or failure</returns>
	bool CalculateViewRects(RECT& priRect, RECT& secRect, RECT& tabRect, RECT& tabbedRect);

	/// <summary>
	/// Window message loop
	/// </summary>
	/// <returns>wParam of last received message</returns>
	WPARAM MessageLoop();

	/// <summary>
	/// Handler function for WM_SIZE message. Set all sub views according to new size
	/// </summary>
	void OnResize();

	/// <summary>
	/// Handler function for menu commands
	/// </summary>
	/// <param name="wParam">Command parameter</param>
	void OnCommand(WPARAM wParam);

	/// <summary>
	/// Handler function to process tab control events
	/// </summary>
	/// <param name="lParam">Additional message information</param>
	void OnNotify(LPARAM lParam);

	/// <summary>
	/// Handler function to process WM_CLOSE message
	/// </summary>
	/// <param name="hWnd">Handle to the window</param>
	/// <param name="wParam">Command parameter</param>
	void OnClose(HWND hWnd, WPARAM wParam);

	/// <summary>
	/// Thread to handle stream events
	/// </summary>
	/// <param name="pThis">Pionter to Kinect window instance</param>
	static DWORD WINAPI StreamEventThread(KinectWindow* pThis);	

private:
	HINSTANCE               m_hInstance;                // Handle to application instance
	HWND                    m_hWndTab;                  // Handle to window of tab control
	HWND                    m_hWndParent;               // Handle to window of main console
	HANDLE                  m_hTimer;                   // Handle to waitable timer
	HANDLE                  m_hThread;                  // Handle to thread instance

	HANDLE					e_hSaveDepthThread;			// Handle to save thread instance
	HANDLE					e_hSaveColorThread;
	HANDLE					e_hSaveSkelThread;
	HANDLE					e_hSaveThread;

	HANDLE					e_hStopSaveDepthThread;		// Handle to stop save thread instance
	HANDLE					e_hStopSaveColorThread;
	HANDLE					e_hStopSaveSkelThread;
	HANDLE					e_hStopSaveThread;

	HANDLE                  m_hStartWindow;             // Handle to the start window sync event
	HANDLE                  m_hStopStreamEventThread;   // Event to stop stream events thread

	KinectSettings*         m_pSettings;                // Pointer to Kinect setting object

	NuiStreamViewer*        m_pPrimaryView;             // Pointer to primary viewer
	NuiStreamViewer*        m_pSecondaryView;           // Pointer to secondary viewer
	NuiAudioViewer*         m_pAudioView;               // Pointer to audio viewer
	NuiAccelerometerViewer* m_pAccelView;               // Pointer to accelerometer viewer
	NuiTiltAngleViewer*     m_pTiltAngleView;           // Pointer to tilt angle viewer
	NuiViewer*              m_pCurTabbedView;           // Pointer to tabbed viewer currently visible
	SaverViewer*            e_pSaveView;                // Pointer to saver viewer

	CameraSettingsViewer*   m_pColorSettingsView;       // Pointer to camera color settings viewer
	CameraSettingsViewer*   m_pExposureSettingsView;    // Pointer to camera exposure settings viewer
	bool                    m_bSupportCameraSettings;   // Indicate whether the sensor supports camera settings

	NuiColorStream*         m_pColorStream;             // Pointer to color stream
	NuiDepthStream*         m_pDepthStream;             // Pointer to depth stream
	NuiSkeletonStream*      m_pSkeletonStream;          // Pointer to skeleton stream
	NuiAudioStream*         m_pAudioStream;             // Pointer to audio stream
	NuiAccelerometerStream* m_pAccelerometerStream;     // Pointer to accelerometer stream			

	INuiSensor*             m_pNuiSensor;               // Pointer to Nui sensor

	std::vector<NuiViewer*>             m_views;        // Collection of Kinect window's sub views
	std::vector<NuiViewer*>             m_tabbedViews;  // Collection of tabbed views
	std::vector<CameraSettingsViewer*>  m_settingViews; // Collection of setting views

	//NuiImageBuffer*						m_imageBuffer;  // Pointer to NuiImageBuffer
	StreamSaver*						e_pStreamSaver;	// Pointer to StreamSaver	

};
