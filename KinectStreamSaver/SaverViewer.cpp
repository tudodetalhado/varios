//------------------------------------------------------------------------------
// <copyright file="StreamSaver.h">
//     Written by Elham Dolatabadi, A PhD candidate at the Univ of Toronto, Toronto Rehab
//------------------------------------------------------------------------------

#include "SaverViewer.h"
#include "StreamSaver.h"

#define DIMAGE        1
#define DRAW		  2
#define CIMAGE        1
#define CRAW		  2

/// Constructor  
SaverViewer::SaverViewer(const NuiViewer* pParent)
	: NuiViewer(pParent)
{
	EnCapture= false;
	EnInitialize = false;
	EnStop= false;
	EnSkel= false;
	EnColor= false;
	EnDepth= false;
	EnDepthStatus = 2;
	EnBG = false;
	EnBGColor = false;
	EnBGDepth = false;

	EnStop_Color= false;
	EnStop_Depth= false;
	EnStop_Skel= false;
	OpenSkelFile = false;
	OpenRawDepth = false;
	OpenRawColor = false;
}

/// Destructor
SaverViewer::~SaverViewer(void)
{
}

/// Dispatch window message to message handlers
LRESULT SaverViewer::DialogProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	switch (uMsg)
	{
	case WM_COMMAND:
		OnCommand(hWnd, wParam);
		break;

	case WM_INITDIALOG:
		OnInit(hWnd);
		break;

	default:
		break;
	}

	return FALSE;
}

/// Returns the ID of the dialog
UINT SaverViewer::GetDlgId()
{
	return IDD_SAVE_VIEW;
}

/// Handler for WM_INITDIALOG message
void SaverViewer::OnInit(HWND hWnd)
{
	// Bind the window handle
	m_hWnd = hWnd;

	CheckDlgButton(hWnd, IDC_Capture, BST_UNCHECKED);
	CheckDlgButton(hWnd, IDC_Stop, BST_UNCHECKED);
}

/// Handler for WM_COMMAND message
void SaverViewer::OnCommand(HWND hWnd, WPARAM wParam)
{
	WORD id   = LOWORD(wParam);
	WORD code = HIWORD(wParam);

	ProcessRecTabCommand(hWnd, code, id);
}

/// Process the StreamViewer Tab
void SaverViewer::ProcessRecTabCommand(HWND hWnd, WORD code, WORD id)
{	
	// Notification Message from Buttons
	if ( code == BN_CLICKED)
	{
		switch (id)
		{
			case IDC_Capture:
			{	
				// Initialize Boolian		
				CaptureInitialize();

				// Retrieve the name of directory
				RetrieveDirectory(hWnd);					

				/// if the check boxes are checked
				if (IsDlgButtonChecked(hWnd, IDC_Skel) == BST_CHECKED)
				{					
					OpenSkelFile = true; // Enable button to open the .binary file
					StringCchPrintfW(FolderName_s,_countof(FolderName_s),L"%s\\Skel", FolderName);	
					if (GetFileAttributes(FolderName_s) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_s,NULL);
					
					EnSkel = true;
					ProcessSkel(hWnd);
				}				

				/// ColorImage
				if (IsDlgButtonChecked(hWnd, IDC_ColorImage) == BST_CHECKED)
				{
					StringCchPrintfW(FolderName_c,_countof(FolderName_c),L"%s\\Color", FolderName);	
					if (GetFileAttributes(FolderName_c) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_c,NULL);
					
					EnColor = true;
					ProcessColorImage(hWnd);
				}

				///ColorBinary
				if (IsDlgButtonChecked(hWnd, IDC_ColorRaw) == BST_CHECKED)
				{
					StringCchPrintfW(FolderName_c,_countof(FolderName_c),L"%s\\Color", FolderName);	
					if (GetFileAttributes(FolderName_c) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_c,NULL);
					
					EnColor = true;
					OpenRawColor = true; // Enable button to open the .binary file
					ProcessColorRaw(hWnd);
				}

				///DepthImage
				if (IsDlgButtonChecked(hWnd, IDC_DepthImage) == BST_CHECKED)
				{
					StringCchPrintfW(FolderName_d,_countof(FolderName_d),L"%s\\Depth", FolderName);	
					if (GetFileAttributes(FolderName_d) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_d,NULL);
					
					EnDepth = true;
					ProcessDepthImage(hWnd);
				}

				///DepthBinary
				if (IsDlgButtonChecked(hWnd, IDC_DepthRaw) == BST_CHECKED)
				{
					StringCchPrintfW(FolderName_d,_countof(FolderName_d),L"%s\\Depth", FolderName);	
					if (GetFileAttributes(FolderName_d) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_d,NULL);
					
					EnDepth = true;
					OpenRawDepth = true;// Enable button to open the .binary file
					ProcessDepthRaw(hWnd);
				}	

				///BackGround
				if (IsDlgButtonChecked(hWnd, IDC_BG) == BST_CHECKED)
				{
					StringCchPrintfW(FolderName_d,_countof(FolderName_d),L"%s\\Depth", FolderName);	
					if (GetFileAttributes(FolderName_d) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_d,NULL);

					StringCchPrintfW(FolderName_c,_countof(FolderName_d),L"%s\\Color", FolderName);	
					if (GetFileAttributes(FolderName_c) == INVALID_FILE_ATTRIBUTES) 
						CreateDirectory(FolderName_c,NULL);
					
					EnBG = true;
					ProcessBG(hWnd);
				}	

				// Fill in the combo boxes
				SetDlgItemText( hWnd, IDC_s2, _T(" ... "));
				SetDlgItemText( hWnd, IDC_r2, _T(" ... "));
				SetDlgItemText( hWnd, IDC_d2, _T(" ... "));
			}
			break;

		case IDC_Stop:
			{
				ProcessStop();				
			}
			break;

		case IDC_BG:
			{
				if (IsDlgButtonChecked(hWnd, IDC_BG) == BST_CHECKED)
				{
					EnBG = true;
					ProcessBG(hWnd);
				}
			} break;

		case IDC_Skel:
			{					
				if (IsDlgButtonChecked(hWnd, IDC_Skel) == BST_CHECKED)
				{
					EnSkel = true;
					ProcessSkel(hWnd);
				}	
			}
			break;

		case IDC_ColorImage:
			{
				if (IsDlgButtonChecked(hWnd, IDC_ColorImage) == BST_CHECKED)
				{
					EnColor = true;
					ProcessColorImage(hWnd);
				}			
			}
			break;

		case IDC_ColorRaw:
			{
				if (IsDlgButtonChecked(hWnd, IDC_ColorRaw) == BST_CHECKED)
				{
					EnColor = true;
					ProcessColorRaw(hWnd);
				}		
			}
			break;		

		case IDC_DepthImage:
			{
				if (IsDlgButtonChecked(hWnd, IDC_DepthImage) == BST_CHECKED)
				{
					EnDepth = true;
					ProcessDepthImage(hWnd);
				}					
			}
			break;

		case IDC_DepthRaw:
			{
				if (IsDlgButtonChecked(hWnd, IDC_DepthRaw) == BST_CHECKED)
				{
					EnDepth = true;
					ProcessDepthRaw(hWnd);
				}	
			}
			break;

		default:
			break;
		}
	}
}

/// a function to set the text of buffering in a Sekeleton dialog box
void SaverViewer::SetSkelBufferingText(const bool Enable)
{
	if (Enable) 
		SetDlgItemText(m_hWnd, IDC_s1, _T("Buffering "));
	else 
		SetDlgItemText(m_hWnd, IDC_s1, _T("Stopped "));
}	

/// a function to set the text of buffering in a Color dialog box
void SaverViewer::SetColorBufferingText(const bool Enable)
{
	if (Enable) 
		SetDlgItemText(m_hWnd, IDC_r1, _T("Buffering "));
	else 
		SetDlgItemText(m_hWnd, IDC_r1, _T("Stopped "));
}

/// a function to set the text of buffering in a Depth dialog box
void SaverViewer::SetDepthBufferingText(const bool Enable)
{
	if (Enable) 
		SetDlgItemText(m_hWnd, IDC_d1, _T("Buffering "));
	else 
		SetDlgItemText(m_hWnd, IDC_d1, _T("Stopped "));
}

/// a function to set the text of recording in a Skeleton dialog box
void SaverViewer::SetSkelRecordingText (const bool Enable)
{
	if (Enable) 
		SetDlgItemText(m_hWnd, IDC_s2, _T("Recording "));
	else 
		SetDlgItemText(m_hWnd, IDC_s2, _T("DONE! "));

}	

/// a function to set the text of recording in a Color dialog box
void SaverViewer::SetColorRecordingText (const bool Enable)
{
	if (Enable) 
		SetDlgItemText(m_hWnd, IDC_r2, _T("Recording "));
	else 
		SetDlgItemText(m_hWnd, IDC_r2, _T("DONE! "));
}

/// a function to set the text of recording in a Depth dialog box
void SaverViewer::SetDepthRecordingText (const bool Enable)
{
	if (Enable) 
		SetDlgItemText( m_hWnd, IDC_d2, _T("Recording "));
	else 
		SetDlgItemText(m_hWnd, IDC_d2, _T("DONE! "));

}

/// a function to set the error textin a dialog box
void SaverViewer::SendErrorText(const int index)
{
	switch (index)
	{
	case 0: // Skeleton Text Box
		SetDlgItemText(m_hWnd, IDC_s2, _T(" Cannot Open CSV File "));
		break;

	case 1: // Color Text Box
		SetDlgItemText(m_hWnd, IDC_r2, _T(" Cannot Open Bitmap File "));
		break;

	case 2: // Depth Text box
		SetDlgItemText(m_hWnd, IDC_d2, _T(" Cannot Open Bitmap File "));
		break;

	default:
		break;
	}
}

void SaverViewer::CaptureInitialize()
{
	EnCapture = true;
	EnInitialize = true;

	EnStop= false;
	EnStop_Color = false;
	EnStop_Depth = false;
	EnStop_Skel = false;
}

void SaverViewer::RetrieveDirectory(HWND hWnd)
{
	LocalDiskName = (LPTSTR)GlobalAlloc(GPTR, MAX_LENGTH_STR);
	FileName1 = (LPTSTR)GlobalAlloc(GPTR, MAX_LENGTH_STR);
	FileName2 = (LPTSTR)GlobalAlloc(GPTR, MAX_LENGTH_STR);


	GetDlgItemText(hWnd,IDC_SubjFolder,LocalDiskName,MAX_LENGTH_STR);
	if (LocalDiskName == NULL || LocalDiskName[0] == 0)
		StringCchPrintfW(FolderName,_countof(FolderName),L"C:\\KinectData");
	else
		StringCchPrintfW(FolderName,_countof(FolderName),L"%s:\\KinectData",LocalDiskName);
	// Create a Directory in a specified local disk name// 
	if (GetFileAttributes(FolderName) == INVALID_FILE_ATTRIBUTES) 
		CreateDirectory(FolderName,NULL);				

	// Create a subfolder in :\KinectData				
	if (GetDlgItemText(hWnd,IDC_TaskFolder1,FileName1,MAX_LENGTH_STR))
	{
		StringCchPrintfW(FolderName,_countof(FolderName),L"%s\\%s", FolderName, FileName1);	
		if (GetFileAttributes(FolderName) == INVALID_FILE_ATTRIBUTES) 
			CreateDirectory(FolderName,NULL);

		// Create a subfolder in :\KinectData\Subj			
		if (GetDlgItemText(hWnd,IDC_TaskFolder2,FileName2,MAX_LENGTH_STR))
		{
			StringCchPrintfW(FolderName,_countof(FolderName),L"%s\\%s", FolderName, FileName2);	
			if (GetFileAttributes(FolderName) == INVALID_FILE_ATTRIBUTES) 
				CreateDirectory(FolderName,NULL);
		}
	}

}

void SaverViewer::ProcessStop()
{
	if (EnColor)
	{
		EnStop_Color = true;
		EnColor = false;
	}

	if (EnDepth)
	{
		EnStop_Depth = true;
		EnDepth = false;
	}

	if (EnSkel)
	{
		EnStop_Skel = true;
		EnSkel = false;
	}
	EnCapture = false;		

	OpenRawColor = false;
	OpenRawDepth = false;
	OpenSkelFile = false;

	EnBG = false;
	EnBGColor = false;
	EnBGDepth = false;
}

void SaverViewer::ProcessBG(HWND hWnd)
{
	EnBG = IsDlgButtonChecked(hWnd, IDC_BG) == BST_CHECKED;
	EnBGColor = true;
	EnBGDepth = true;
	CheckDlgButton(hWnd, IDC_Skel, BST_UNCHECKED);
	CheckDlgButton(hWnd, IDC_ColorRaw, BST_UNCHECKED);
	CheckDlgButton(hWnd, IDC_DepthRaw, BST_UNCHECKED);
	CheckDlgButton(hWnd, IDC_ColorImage, BST_UNCHECKED);
	CheckDlgButton(hWnd, IDC_DepthImage, BST_UNCHECKED);

}

void SaverViewer::ProcessColorImage(HWND hWnd)
{	
	EnColorStatus = CIMAGE;
	CheckDlgButton(hWnd, IDC_BG, BST_UNCHECKED);
	CheckDlgButton(hWnd, IDC_ColorRaw, BST_UNCHECKED);			
}

void SaverViewer::ProcessColorRaw(HWND hWnd)
{
	EnColorStatus = CRAW;
	CheckDlgButton(hWnd, IDC_BG, BST_UNCHECKED);
	CheckDlgButton(hWnd, IDC_ColorImage, BST_UNCHECKED);		
}

void SaverViewer::ProcessDepthImage(HWND hWnd)
{
	EnDepthStatus = DIMAGE;
	CheckDlgButton(hWnd, IDC_BG, BST_UNCHECKED);
	CheckDlgButton(hWnd, IDC_DepthRaw, BST_UNCHECKED);	
}

void SaverViewer::ProcessDepthRaw(HWND hWnd)
{
	EnDepthStatus = DRAW;
	CheckDlgButton(hWnd, IDC_BG, BST_UNCHECKED);
	CheckDlgButton(hWnd, IDC_DepthImage, BST_UNCHECKED);
}

void SaverViewer::ProcessSkel(HWND hWnd)
{
	CheckDlgButton(hWnd, IDC_BG, BST_UNCHECKED);		
}