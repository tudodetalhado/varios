//------------------------------------------------------------------------------
// <copyright file="StreamSaver.h">
//     Written by Elham Dolatabadi, A PhD candidate at the Univ of Toronto, Toronto Rehab
//									A memeber of IATSL - Intelligent Assistive Technology and Systems Lab
//------------------------------------------------------------------------------


#include "StreamSaver.h"
#include "SaverViewer.h"


#define SKEL         0
#define Color		 1
#define DEPTH        2

#define STREAM       1
#define BG			 2

/// Constructor
StreamSaver::StreamSaver(INuiSensor* pNuiSensor)			
{
	InitializeStreamSaver();	
}

/// Destructor
StreamSaver::~StreamSaver()
{
	UnInitializeStreamSaver();
}

/// Initialization
void StreamSaver::InitializeStreamSaver()
{
	InitializeCriticalSectionAndSpinCount(&CriticalSection_Color,0x00000400);
	InitializeCriticalSectionAndSpinCount(&CriticalSection_Depth,0x00000400);
	InitializeCriticalSectionAndSpinCount(&CriticalSection_Skeleton,0x00000400);

	s_Counter = 0;
	c_Counter= 0; cc_Counter = 0;
	d_Counter= 0;dd_Counter = 0;
	c_old = 0;d_old = 0;	
	cOld_frame=0;
	dOld_frame=0;
	AllocatePointer();		
}

/// Allocation
void StreamSaver::AllocatePointer()
{
	ColorBuffer = new ImageStream;
	DepthBuffer = new ImageStream;
	SkeletonBuffer = new SkeletonStream;

	EnInitializeColor = false;
	EnInitializeDepth = false;
	EnInitializeSkel = false;
}

/// UnInitialization
void StreamSaver::UnInitializeStreamSaver()
{
	if(ColorBuffer)
	{
		delete ColorBuffer;
		ColorBuffer = nullptr;			
	}

	if(DepthBuffer)
	{
		delete DepthBuffer;
		DepthBuffer = nullptr;			
	}

	if(SkeletonBuffer)
	{
		delete SkeletonBuffer;
		SkeletonBuffer = nullptr;			
	}

	// Release resources used by the critical section object.
	DeleteCriticalSection(&CriticalSection_Color);
	DeleteCriticalSection(&CriticalSection_Depth);
	DeleteCriticalSection(&CriticalSection_Skeleton);
}

/// Push back the Skeleton streams into a buffer
void StreamSaver::BufferSkeletonStream (const NUI_SKELETON_FRAME* pFrame)
{
	const NUI_SKELETON_FRAME*		nui_skeleton_frame; // Pointer to Skeleton Buffer
	nui_skeleton_frame = pFrame;

	if (EnInitializeSkel)
	{
		SkeletonBuffer = new SkeletonStream; 
		EnInitializeSkel = false;
	}	

	// Request ownership of the critical section.
	EnterCriticalSection(&CriticalSection_Skeleton);

	for (int i = 0 ; i < NUI_SKELETON_COUNT ; i++)
	{
		SkeletonBuffer->SkeletonJointPosition[i].push_back(nui_skeleton_frame->SkeletonData[i]);
	}
	SkeletonBuffer->FrameTime.push_back(nui_skeleton_frame->liTimeStamp);
	SkeletonBuffer->dwframenumber.push_back(nui_skeleton_frame->dwFrameNumber);

	// Release ownership of the critical section.
	LeaveCriticalSection(&CriticalSection_Skeleton);
} 

/// Push back the Color streams into a buffer
void StreamSaver::BufferColorStream(const NuiImageBuffer* pImage, 
	DWORD ColorFrameNumber,
	LARGE_INTEGER	ColorliTimeStamp)
{
	const NuiImageBuffer*			nui_Buffer; // Pointer to Image Buffer
	nui_Buffer =					pImage;
	shared_ptr<vector <BYTE>>		Buffer(new vector<BYTE>(nui_Buffer->GetBufferSize()));
	copy ( nui_Buffer->GetBuffer(), 
		nui_Buffer->GetBuffer()+nui_Buffer->GetBufferSize(), 
		Buffer->begin());	

	if (EnInitializeColor)
	{
		ColorBuffer = new ImageStream; 
		EnInitializeColor = false;
	}	

	// Request ownership of the critical section.
	EnterCriticalSection(&CriticalSection_Color);

	ColorBuffer->height.push_back(nui_Buffer->GetHeight());
	ColorBuffer->width.push_back(nui_Buffer->GetWidth());
	ColorBuffer->FrameNumber.push_back(ColorFrameNumber);
	ColorBuffer->FrameTime.push_back(ColorliTimeStamp);
	ColorBuffer->ImageBuffer.push_back(Buffer);
	
	// Release ownership of the critical section.
	LeaveCriticalSection(&CriticalSection_Color);

}

/// Push back the Depth streams into a buffer
void StreamSaver::BufferDepthStream (const NuiImageBuffer* pImage,
	DWORD DepthFrameNumber,
	LARGE_INTEGER DepthliTimeStamp,
	USHORT* DepthValue){

		const NuiImageBuffer*			nui_Buffer; // Pointer to Image Buffer
		nui_Buffer =					pImage;
		shared_ptr<vector <BYTE>>		Buffer(new vector<BYTE>(nui_Buffer->GetBufferSize()));

		copy ( nui_Buffer->GetBuffer(), 
			nui_Buffer->GetBuffer()+nui_Buffer->GetBufferSize(), 
			Buffer->begin());	

		shared_ptr<vector <USHORT>>		PointCloud(new vector<USHORT>(640*480));
		copy ( DepthValue, 
			DepthValue+640*480, 
			PointCloud->begin());	

		if (EnInitializeDepth)
		{
			DepthBuffer = new ImageStream; 
			EnInitializeDepth = false;
		}	
		// Request ownership of the critical section.
		EnterCriticalSection(&CriticalSection_Depth);

		DepthBuffer->height.push_back(nui_Buffer->GetHeight());
		DepthBuffer->width.push_back(nui_Buffer->GetWidth());
		DepthBuffer->FrameNumber.push_back(DepthFrameNumber);
		DepthBuffer->FrameTime.push_back(DepthliTimeStamp);
		DepthBuffer->ImageBuffer.push_back(Buffer);
		RawDepthBuffer.push_back(PointCloud);

		// Release ownership of the critical section.
		LeaveCriticalSection(&CriticalSection_Depth);
}

/// Save Skeleton streams as .binary files
int StreamSaver:: SaveSkeletonStream(bool EnStop_Skel)
{
	bool EnSave = false;
	int Status = 4;	

	NUI_SKELETON_DATA TempSkeletonBuffer[NUI_SKELETON_COUNT];
	DWORD TempframeNumber;
	LARGE_INTEGER TempFrameTime;

	NUI_SKELETON_BONE_ORIENTATION boneOrientations[NUI_SKELETON_POSITION_COUNT]; //holds joint angles for 1 skeleton
	Vector4 quaternion, jointposition = {0,0,0,1}; //Vector4 field order: x,y,z,w
//	Vector4 joint[20];
	HRESULT boneStatus;

	if (SkeletonBuffer)
	{
		// Request ownership of the critical section.
		EnterCriticalSection(&CriticalSection_Skeleton);

		if (!SkeletonBuffer->dwframenumber.empty())
		{
			for (int i = 0 ; i < NUI_SKELETON_COUNT ; i++)
			{
				TempSkeletonBuffer[i] = SkeletonBuffer->SkeletonJointPosition[i].front();			
				SkeletonBuffer->SkeletonJointPosition[i].pop_front(); // remove the first elements in the buffer
				SkeletonBuffer->SkeletonJointPosition[i].shrink_to_fit();
			}
			TempframeNumber = SkeletonBuffer->dwframenumber.front();
			TempFrameTime = SkeletonBuffer->FrameTime.front();

			SkeletonBuffer->dwframenumber.pop_front(); // remove the first elements in the buffer
			SkeletonBuffer->dwframenumber.shrink_to_fit();
			SkeletonBuffer->FrameTime.pop_front(); // remove the first elements in the buffer
			SkeletonBuffer->FrameTime.shrink_to_fit();

			s_Counter++;

			EnSave = true;
		}

		// Release ownership of the critical section.
		LeaveCriticalSection(&CriticalSection_Skeleton);	

		if (EnSave)
		{		
			float jbuffer[NUI_SKELETON_POSITION_COUNT*NUI_SKELETON_COUNT][4]={0,0};	
			float qbuffer[NUI_SKELETON_POSITION_COUNT*NUI_SKELETON_COUNT][4]={0,0};	

			for (int i = 0 ; i < NUI_SKELETON_COUNT ; i++)
			{	
				boneStatus = NuiSkeletonCalculateBoneOrientations(&TempSkeletonBuffer[i], boneOrientations); 
				for (int j = 0 ; j < NUI_SKELETON_POSITION_COUNT ; j++ )
				{
					int k = i*20+j;
					// jointPosition
					jbuffer[k][0] = TempSkeletonBuffer[i].SkeletonPositions[j].x;
					jbuffer[k][1] = TempSkeletonBuffer[i].SkeletonPositions[j].y;
					jbuffer[k][2] = TempSkeletonBuffer[i].SkeletonPositions[j].z;	
					jbuffer[k][3] = TempSkeletonBuffer[i].eSkeletonPositionTrackingState[j];

					// jointOrientation
					quaternion = boneOrientations[j].hierarchicalRotation.rotationQuaternion;
					qbuffer[k][0] = quaternion.w;
					qbuffer[k][1] = quaternion.y;
					qbuffer[k][2] = quaternion.z;	
					qbuffer[k][3] = quaternion.x;
				}
			}	

			//
			LONGLONG tbuffer[5]={0};	
			//LONGLONG tbuffer;
				
			tbuffer[0] = s_Counter;
			tbuffer[1] = TempframeNumber;
			tbuffer[2] = TempFrameTime.HighPart;
			tbuffer[3] = TempFrameTime.LowPart;
			tbuffer[4] = TempFrameTime.QuadPart;

			if (SkeletonJoint || SkeletonTime)
			{
				SkeletonJoint.write((const char*)(jbuffer), NUI_SKELETON_COUNT*NUI_SKELETON_POSITION_COUNT*4*sizeof(float));
				SkeletonOrient.write((const char*)(qbuffer), NUI_SKELETON_COUNT*NUI_SKELETON_POSITION_COUNT*4*sizeof(float));
				SkeletonTime.write((const char*)(&tbuffer),5*sizeof(LONGLONG));		
				Status=1;
			} else Status=0;

			EnSave = false;
		}
		else if (EnStop_Skel)
		{
			if (SkeletonBuffer->dwframenumber.empty())
			{
				EnStop_Skel = StopSaveSkeleton(EnStop_Skel);		
				Status=2;
			}
		}
	}
	return Status;
}

/// Save Color Streams 
int StreamSaver::SaveColorStream(bool EnStop_Color, WCHAR foldername[MAX_PATH], int EnColorStatus)
{
	bool EnSave = false;	
	int Status = 4;

	if (ColorBuffer)
	{
		shared_ptr<vector <BYTE>>	TempColorBuffer(new vector<BYTE>(ColorBuffer->ImageBuffer.size()));
	
		DWORD						TempHeight;
		DWORD						TempWidth;
		DWORD						TempFrameNumber;
		LARGE_INTEGER				TempTime;		

		// Request ownership of the critical section.
		EnterCriticalSection(&CriticalSection_Color);

		if (! ColorBuffer->ImageBuffer.empty())
		{			
			TempColorBuffer = ColorBuffer->ImageBuffer.front();
			TempHeight =  ColorBuffer->height.front();
			TempWidth =  ColorBuffer->width.front();
			TempFrameNumber = ColorBuffer->FrameNumber.front();
			TempTime = ColorBuffer->FrameTime.front();

			Pop_FrontColor();

			EnSave = true;
		}		

		// Release ownership of the critical section.
		LeaveCriticalSection(&CriticalSection_Color);
	
		if (EnSave)
		{
			//c_Counter ++;
			//cOld_frame++;
			//if (cOld_frame == 2 )
			//{
				c_Counter ++;
				if (EnColorStatus == 1)
				{
					WCHAR c_screenshotPath[MAX_PATH];						
					StringCchPrintfW (c_screenshotPath,_countof(c_screenshotPath),L"%s\\%ld_Color.bmp",foldername,c_Counter);				
					HRESULT hr = SaveColorBitmapToFile (TempColorBuffer->data(),TempWidth,TempHeight,32,c_screenshotPath);
					if (SUCCEEDED(hr))	Status=1;							
					else Status=0;
				}	else if (EnColorStatus == 2)
				{				
					HRESULT hr = SaveRawColor(TempColorBuffer->data(),TempWidth,TempHeight,32,foldername,c_Counter);
					if (SUCCEEDED(hr))	Status=1;							
					else Status=0;
				}
				cOld_frame = 0;
			//}			
			EnSave = false;	
		}
		else if (EnStop_Color)
		{
			if (ColorBuffer->ImageBuffer.empty())
			{
				EnStop_Color = StopSaveColor(EnStop_Color);
				Status=2;
			}
		}
	}
	
	return Status;
}

/// Save Depth Streams 
int StreamSaver::SaveDepthStream(bool EnStop_Depth, WCHAR foldername[MAX_PATH],int EnDepthStatus)
{
	bool EnSave = false;	
	int Status = 4;

	if (DepthBuffer)
	{
		shared_ptr<vector <BYTE>> TempBuffer (new vector<BYTE>(DepthBuffer->ImageBuffer.size()));
		DWORD TempHeight;
		DWORD TempWidth;
		DWORD TempFrameNumber;
		LARGE_INTEGER TempTime;
		shared_ptr<vector <USHORT>> TempdepthPC (new vector<USHORT>(RawDepthBuffer.size()));	

		// Request ownership of the critical section.
		EnterCriticalSection(&CriticalSection_Depth);
		if (! DepthBuffer->ImageBuffer.empty())
		{			
			TempBuffer = DepthBuffer->ImageBuffer.front();
			TempHeight =  DepthBuffer->height.front();
			TempWidth =  DepthBuffer->width.front();
			TempFrameNumber = DepthBuffer->FrameNumber.front();
			TempTime = DepthBuffer->FrameTime.front();
			TempdepthPC = RawDepthBuffer.front();	

			Pop_FrontDepth();

			EnSave = true;			
		}
		// Release ownership of the critical section.
		LeaveCriticalSection(&CriticalSection_Depth);

		if (EnSave)
		{		
			d_Counter ++;
			if (EnDepthStatus == 1)
			{
				WCHAR d_screenshotPath[MAX_PATH];
				StringCchPrintfW (d_screenshotPath,_countof(d_screenshotPath),L"%s\\%ld_depth.bmp",foldername,d_Counter);	

				HRESULT hr = SaveDepthBitmapToFile (TempBuffer->data(),TempWidth,TempHeight, 32, d_screenshotPath);
				if (FAILED(hr))			
					Status=0;			
				else Status=1;
			}else
			{
				HRESULT hr = SaveRawDepth(TempdepthPC->data(),foldername,d_Counter);
				if (FAILED(hr))	Status=0;							
				else Status=1;
			}
			dOld_frame = 0;				
			EnSave = false;
		}
		else if (EnStop_Depth)
		{
			if (DepthBuffer->ImageBuffer.empty())
			{			
				EnStop_Depth = StopSaveDepth(EnStop_Depth);
				Status=2;		
			}
		}
	}
	return Status;
}

/// Save Bitmap images to a file
HRESULT StreamSaver::SaveColorBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{

	DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

	BITMAPINFOHEADER bmpInfoHeader = {0};

	bmpInfoHeader.biSize        = sizeof(BITMAPINFOHEADER);  // Size of the header
	bmpInfoHeader.biBitCount    = wBitsPerPixel;             // Bit count
	bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
	bmpInfoHeader.biWidth       = lWidth;                    // Width in pixels
	bmpInfoHeader.biHeight      = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
	bmpInfoHeader.biPlanes      = 1;                         // Default
	bmpInfoHeader.biSizeImage   = dwByteCount;               // Image size in bytes

	BITMAPFILEHEADER bfh = {0};

	bfh.bfType    = 0x4D42;                                           // 'M''B', indicates bitmap
	bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
	bfh.bfSize    = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

	// Create the file on disk to write to
	HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);	

	// Return if error opening file
	if (NULL == hFile) 
	{
		return E_ACCESSDENIED;
	}

	DWORD dwBytesWritten = 0;

	// Write the bitmap file header
	if ( !WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL) )
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the bitmap info header
	if ( !WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL) )
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the Color Data
	if ( !WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL) )
	{
		CloseHandle(hFile);
		return E_FAIL;
	}   

	// Close the file
	CloseHandle(hFile);
	return S_OK;
}

/// Save Bitmap images to a file
HRESULT StreamSaver::SaveDepthBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{

	DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

	BITMAPINFOHEADER bmpInfoHeader = {0};

	bmpInfoHeader.biSize        = sizeof(BITMAPINFOHEADER);  // Size of the header
	bmpInfoHeader.biBitCount    = wBitsPerPixel;             // Bit count
	bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
	bmpInfoHeader.biWidth       = lWidth;                    // Width in pixels
	bmpInfoHeader.biHeight      = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
	bmpInfoHeader.biPlanes      = 1;                         // Default
	bmpInfoHeader.biSizeImage   = dwByteCount;               // Image size in bytes

	BITMAPFILEHEADER bfh = {0};

	bfh.bfType    = 0x4D42;                                           // 'M''B', indicates bitmap
	bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
	bfh.bfSize    = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

	// Create the file on disk to write to
	HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);	

	// Return if error opening file
	if (NULL == hFile) 
	{
		return E_ACCESSDENIED;
	}

	DWORD dwBytesWritten = 0;

	// Write the bitmap file header
	if ( !WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL) )
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the bitmap info header
	if ( !WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL) )
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the Color Data
	if ( !WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL) )
	{
		CloseHandle(hFile);
		return E_FAIL;
	}   

	// Close the file
	CloseHandle(hFile);
	return S_OK;
}

/// Save the binary images to a file
HRESULT StreamSaver:: SaveRawColor(BYTE* RGBImage, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, WCHAR foldername[MAX_PATH], int Counter)
{	
	if (Counter-c_old>200)
	{
		cc_Counter++;
		RawColor.close();
		OpenRawColorFile(foldername, cc_Counter);
		c_old = Counter;		
	}


	DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);
	RawColor.write((const char*)(RGBImage), dwByteCount*sizeof(BYTE));			
	return S_OK;
}

/// Save the binary images to a file
HRESULT StreamSaver:: SaveRawDepth(USHORT usDepthValue[640*480], WCHAR foldername[MAX_PATH],int Counter)
{
	if (Counter-d_old>200)
	{
		dd_Counter++;
		RawDepth.close();
		OpenRawDepthFile(foldername, dd_Counter);
		d_old = Counter;		
	}

	RawDepth.write((const char*)(usDepthValue), 640*480*sizeof(USHORT));
	return S_OK;	
}

/// Stop the Saving the streams
bool StreamSaver::StopSaveColor(bool SaveStatus)
{
	SaveStatus = false;

	EnterCriticalSection(&CriticalSection_Color); // Request ownership of the critical section.
	ClearColorBuffer(); // Clear the buffer.		
	LeaveCriticalSection(&CriticalSection_Color); // Release ownership of the critical section.	

	RawColor.close();
	delete ColorBuffer;
	ColorBuffer = nullptr;
	EnInitializeColor = true;
	c_Counter= 0; cc_Counter = 0;
	c_old = 0;	
	return SaveStatus;
}

/// Stop the Saving the streams
bool StreamSaver::StopSaveDepth(bool SaveStatus)
{
	SaveStatus = false;

	EnterCriticalSection(&CriticalSection_Depth); // Request ownership of the critical section.
	ClearDepthBuffer(); // Clear the buffer.		
	LeaveCriticalSection(&CriticalSection_Depth); // Release ownership of the critical section.	

	RawDepth.close();	
	delete DepthBuffer;
	DepthBuffer = nullptr;
	EnInitializeDepth = true;
	d_Counter= 0;dd_Counter = 0;
	d_old = 0;	
	return SaveStatus;
}

/// Stop the Saving the streams
bool StreamSaver::StopSaveSkeleton(bool SaveStatus)
{
	SaveStatus = false;

	EnterCriticalSection(&CriticalSection_Skeleton); // Request ownership of the critical section.
	ClearSkeletonBuffer(); // Clear the buffer.		
	LeaveCriticalSection(&CriticalSection_Skeleton); // Release ownership of the critical section.	

	SkeletonJoint.close();
	SkeletonOrient.close();		
	SkeletonTime.close();

	delete SkeletonBuffer;
	SkeletonBuffer = nullptr;
	EnInitializeSkel = true;
	s_Counter = 0;
	return SaveStatus;
}

/// Open a .binary file to save the skeleton streams
void StreamSaver:: OpenSkeletonFile(WCHAR foldername[MAX_PATH])
{
	WCHAR SkeletonFileName[MAX_PATH];						
	StringCchPrintfW(SkeletonFileName,_countof(SkeletonFileName),L"%s\\Joint_Position.binary", foldername);
	SkeletonJoint.open(SkeletonFileName, ios::binary);

	StringCchPrintfW(SkeletonFileName,_countof(SkeletonFileName),L"%s\\Joint_Orientation.binary", foldername);
	SkeletonOrient.open(SkeletonFileName, ios::binary);

	StringCchPrintfW(SkeletonFileName,_countof(SkeletonFileName),L"%s\\liTimeStamp.binary", foldername);
	SkeletonTime.open(SkeletonFileName, ios::binary);			

}

/// Open a .binary file to save the raw color streams
void StreamSaver:: OpenRawColorFile(WCHAR foldername[MAX_PATH], int counter)
{
	WCHAR PointCloudFileName[MAX_PATH];						
	StringCchPrintfW(PointCloudFileName,_countof(PointCloudFileName),L"%s\\Color_Raw_%ld.binary", foldername,counter);
	RawColor.open(PointCloudFileName, ios::binary);
}

/// Open a .binary file to save the raw depth streams
void StreamSaver:: OpenRawDepthFile(WCHAR foldername[MAX_PATH], int counter)
{
	WCHAR PointCloudFileName[MAX_PATH];						
	StringCchPrintfW(PointCloudFileName,_countof(PointCloudFileName),L"%s\\Depth_Raw_%ld.binary", foldername,counter);
	RawDepth.open(PointCloudFileName, ios::binary);	
}

/// create a pointcloud matrix containg depth pixles transfomed to skeleton space 
void StreamSaver:: CreatePointCloud(LONG DepthX, LONG DepthY, USHORT usDepthValue[640*480],WCHAR foldername[MAX_PATH])
{
	WCHAR BGFileName[MAX_PATH];						
	StringCchPrintfW(BGFileName,_countof(BGFileName),L"%s\\BackGround.csv", foldername);
	BGpointCloud.open(BGFileName);

	int i = 0;
	for( int y = 0 ; y < 480 ; y++ ) 
	{
		for( int x = 0 ; x < 640 ; x++ )
		{
			//USHORT depth = NuiDepthPixelToDepth(usDepthValue[i]);
			//Vector4 v =	NuiTransformDepthImageToSkeleton(x,y, NuiDepthPixelToDepth(usDepthValue[i]));
			Vector4 v =	NuiTransformDepthImageToSkeleton(x,y, NuiDepthPixelToDepth(usDepthValue[i]), NUI_IMAGE_RESOLUTION_640x480);
			BGpointCloud << fixed << v.x << "," 
				<< fixed << v.y << "," 
				<< fixed << v.z << "\n";
			i++;
		}
	}
	BGpointCloud.close();

}

/// Removes the first element in the Color Stream Buffer
void StreamSaver:: Pop_FrontColor()
{
	ColorBuffer->ImageBuffer.pop_front();			
	ColorBuffer->height.pop_front();			
	ColorBuffer->width.pop_front();				
	ColorBuffer->FrameNumber.pop_front();				
	ColorBuffer->FrameTime.pop_front();

	ColorBuffer->ImageBuffer.shrink_to_fit();			
	ColorBuffer->height.shrink_to_fit();
	ColorBuffer->width.shrink_to_fit();
	ColorBuffer->FrameNumber.shrink_to_fit();
	ColorBuffer->FrameTime.shrink_to_fit();
}

/// Removes the first element in the Depth Stream Buffer
void StreamSaver:: Pop_FrontDepth()
{
	DepthBuffer->ImageBuffer.pop_front();	
	DepthBuffer->height.pop_front();	
	DepthBuffer->width.pop_front();
	DepthBuffer->FrameNumber.pop_front();
	DepthBuffer->FrameTime.pop_front();
	RawDepthBuffer.pop_front();

	DepthBuffer->ImageBuffer.shrink_to_fit();
	DepthBuffer->height.shrink_to_fit();
	DepthBuffer->width.shrink_to_fit();
	DepthBuffer->FrameNumber.shrink_to_fit();
	DepthBuffer->FrameTime.shrink_to_fit();
	RawDepthBuffer.shrink_to_fit();

}

/// Clear the Skeleton Stream Buffer
void StreamSaver::ClearSkeletonBuffer()
{
	SkeletonBuffer->dwframenumber.clear();
	SkeletonBuffer->dwframenumber.shrink_to_fit();
	SkeletonBuffer->FrameTime.clear();
	SkeletonBuffer->FrameTime.shrink_to_fit();

	for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
	{
		SkeletonBuffer->SkeletonJointPosition[i].clear();
		SkeletonBuffer->SkeletonJointPosition[i].shrink_to_fit();
	}	
}

/// Clear the Color Stream Buffer
void StreamSaver::ClearColorBuffer()
{
	ColorBuffer->ImageBuffer.clear();
	ColorBuffer->height.clear();
	ColorBuffer->width.clear();

	ColorBuffer->ImageBuffer.shrink_to_fit();
	ColorBuffer->height.shrink_to_fit();
	ColorBuffer->width.shrink_to_fit();	
}

/// Clear the Depth Stream Buffer
void StreamSaver::ClearDepthBuffer()
{
	DepthBuffer->ImageBuffer.clear();
	DepthBuffer->height.clear();
	DepthBuffer->width.clear();
	RawDepthBuffer.clear();

	DepthBuffer->ImageBuffer.shrink_to_fit();
	DepthBuffer->height.shrink_to_fit();
	DepthBuffer->width.shrink_to_fit();	
	RawDepthBuffer.shrink_to_fit();	
}

