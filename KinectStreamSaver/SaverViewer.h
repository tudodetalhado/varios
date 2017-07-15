//------------------------------------------------------------------------------
// <copyright file="StreamSaver.h">
//     Written by Elham Dolatabadi, A PhD candidate at the Univ of Toronto, Toronto Rehab
//------------------------------------------------------------------------------


#pragma once

#include "NuiViewer.h"
#include "resource.h"
#include "stdafx.h"
#include <stdio.h>
#include <tchar.h>
#include <strsafe.h>

using namespace std;

#define	MAX_LENGTH_STR	500

class KinectWindow;
class StreamSaver;

class SaverViewer : public NuiViewer
{
public:	

	/// Constructor    
	SaverViewer(const NuiViewer* pParent);

	/// Destructor
	~SaverViewer(void);	

	/// a function to set the text of recording in a dialog box
	void			SetSkelRecordingText(const bool Enable);
	void			SetColorRecordingText(const bool Enable);
	void			SetDepthRecordingText(const bool Enable);	

	/// set the error textin a dialog box
	void			SendErrorText(const int index);

	/// set the text of buffering in a dialog box
	void			SetSkelBufferingText(const bool Enable);
	void			SetColorBufferingText(const bool Enable);
	void			SetDepthBufferingText(const bool Enable);	

	bool			EnCapture;	
	bool			EnInitialize;
	bool			EnSkel;
	bool			EnColor;
	bool			EnDepth;
	int				EnDepthStatus;
	int				EnColorStatus;
	bool			EnBG;
	bool			EnBGColor;
	bool			EnBGDepth;

	bool			EnStop;
	bool			EnStop_Color;
	bool			EnStop_Depth;
	bool			EnStop_Skel;

	bool			OpenSkelFile;
	bool			OpenRawDepth;
	bool			OpenRawColor;

	WCHAR			FolderName[MAX_PATH]; // create a folder in C://KinectData
	WCHAR			FolderName_s[MAX_PATH]; // create a color folder; C://KinectData//FolderName//skel
	WCHAR			FolderName_c[MAX_PATH]; // create a color folder; C://KinectData//FolderName//color
	WCHAR			FolderName_d[MAX_PATH]; // create a depth folder; C://KinectData//FolderName//depth

private:

	/// Dispatch window message to message handlers
	virtual		LRESULT DialogProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// Returns the ID of the dialog
	virtual		UINT GetDlgId();

	/// Handler for WM_INITDIALOG message
	void		OnInit(HWND hWnd);

	/// Handler for WM_COMMAND message
	void		OnCommand(HWND hWnd, WPARAM wParam);

	/// Process the StreamViewer Tab
	void		ProcessRecTabCommand(HWND hWnd,WORD code, WORD id);

	/// Release all the resources
	void		CleanUp();

	/// Initialization
	void		CaptureInitialize();

	/// Retrieve the directory entered by user
	void		RetrieveDirectory(HWND hWnd);

	/// Process the checkboxes
	void		ProcessStop();
	void		ProcessBG(HWND hWnd);
	void		ProcessSkel(HWND hWnd);
	void		ProcessColor(HWND hWnd);
	void		ProcessColorImage(HWND hWnd);
	void		ProcessColorRaw(HWND hWnd);
	void		ProcessDepth(HWND hWnd);
	void		ProcessDepthImage(HWND hWnd);
	void		ProcessDepthRaw(HWND hWnd);

	LPTSTR		LocalDiskName; 
	LPTSTR		FileName1;
	LPTSTR		FileName2;

};

