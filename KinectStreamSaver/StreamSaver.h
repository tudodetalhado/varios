//------------------------------------------------------------------------------
// <copyright file="StreamSaver.h">
//     Written by Elham Dolatabadi, A PhD candidate at the Univ of Toronto, Toronto Rehab
//									A memeber of IATSL - Intelligent Assistive Technology and Systems Lab
//------------------------------------------------------------------------------

#pragma once

#include "stdafx.h"
#include <stdio.h>
#include "NuiImageBuffer.h"
#include <vector>
#include <deque>
#include <array>
#include "resource.h"
#include <fstream>
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream
#include <memory>

using namespace std;

class SaverViewer;

#define	MAX_LENGTH_STR	500

class StreamSaver 
{
private:
	/// Initialization
	void							InitializeStreamSaver();
	/// UnInitialization
	void							UnInitializeStreamSaver();
	/// Allocataion
	void							AllocatePointer();
	/// Save Bitmap images to a file
	HRESULT							SaveColorBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCTSTR lpszFilePath);
	HRESULT							SaveDepthBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCTSTR lpszFilePath);
	HRESULT							SaveColorPNGToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCTSTR lpszFilePath);

	/// Get the name of the file where screenshot will be stored
	HRESULT							GetScreenshotFileName(wchar_t *screenshotName, UINT screenshotNameSize);

	/// Save the binary images to a file
	HRESULT							SaveRawDepth(USHORT usDepthValue[640*480], WCHAR foldername[MAX_PATH],int counter);
	HRESULT							SaveRawColor(BYTE* p, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, WCHAR foldername[MAX_PATH], int counter);

	/// Stop the Saving the streams
	bool							StopSaveColor(bool StopStatus);	
	bool							StopSaveDepth(bool StopStatus);
	bool							StopSaveSkeleton(bool StopStatus);

	/// Removes the first element in the deque container
	void							Pop_FrontColor();
	void							Pop_FrontDepth();

	/// Clear the Buffer
	void							ClearSkeletonBuffer();
	void							ClearColorBuffer();
	void							ClearDepthBuffer();

	CRITICAL_SECTION				CriticalSection_Skeleton; // Variable to Critical Section object
	CRITICAL_SECTION				CriticalSection_Color; // Variable to Critical Section object
	CRITICAL_SECTION				CriticalSection_Depth; // Variable to Critical Section object
	
	struct	SkeletonStream // Dynamic Buufer to store Skeleton streams
	{  
		deque <NUI_SKELETON_DATA> SkeletonJointPosition[NUI_SKELETON_COUNT];
		deque <DWORD> dwframenumber;
		deque <LARGE_INTEGER> FrameTime;
	};
	SkeletonStream*					SkeletonBuffer;	

	struct ImageStream // Dynamic Buufer to store color streams
	{
		deque <shared_ptr<vector <BYTE> > >				ImageBuffer;
		deque <DWORD >									width;
		deque <DWORD >									height;
		deque <DWORD >									FrameNumber;
		deque <LARGE_INTEGER>							FrameTime;
	};

	ImageStream*										 ColorBuffer;
	ImageStream*										 DepthBuffer;

	deque <shared_ptr<vector <USHORT> > >				RawDepthBuffer;		
	deque <shared_ptr<vector <USHORT> > >				TestDepthBuffer;		

	ofstream						SkeletonJoint;		// .binary file to write the skeletonjoint
	ofstream						SkeletonOrient;		// .binary file to write the skeletonOrient
	ofstream						SkeletonTime;		// .binary file to write the timestamp	
	ofstream						BGpointCloud;		// .binary file to write the pointcloud
	ofstream						RawDepth;			// .binary file to write the binary depth
	ofstream						RawColor;			// .binary file to write the binary color

	bool							EnInitializeColor;
	bool							EnInitializeDepth;	
	bool							EnInitializeSkel;
	int								s_Counter;	
	int								c_Counter,cc_Counter;	
	int								c_old,d_old;
	DWORD							cOld_frame;			// old timestamp
	DWORD							dOld_frame;	
	int								d_Counter,dd_Counter;								

public:
	/// Constructor
	StreamSaver(INuiSensor* pNuiSensor);

	/// Destructor
	~StreamSaver();

	// Save Skeleton Streams 
	int								SaveSkeletonStream(bool EnStop_Skel);

	// Save Color Streams 
	int								SaveColorStream(bool EnStop_Color, WCHAR foldername[MAX_PATH], int EnColorStatus);

	// Save Depth Streams 
	int								SaveDepthStream(bool EnStop_Depth, WCHAR foldername[MAX_PATH], int EnDepthStatus);

	
	// Push back the Skeleton streams into a buffer
	void							BufferSkeletonStream(const NUI_SKELETON_FRAME* pFrame);

	// Push back the Color streams into a buffer
	void							BufferColorStream(const NuiImageBuffer* pImage, DWORD ColorFrameNumber,LARGE_INTEGER ColorliTimeStamp);

	// Push back the Depth streams into a buffer
	void							BufferDepthStream(const NuiImageBuffer* pImage, DWORD DepthFrameNumber,LARGE_INTEGER DepthliTimeStamp,USHORT* DepthValue);	

	// create a CreatePointCloud matrix containg depth pixles transfomed to skeleton space 
	void							CreatePointCloud(LONG DepthX, LONG DepthY, USHORT usDepthValue[640*480],WCHAR foldername[MAX_PATH]);

	// Open a .binary file to save the skeleton streams
	void							OpenSkeletonFile(WCHAR foldername[MAX_PATH]);

	// Open a .binary file to save raw depth streams
	void							OpenRawDepthFile(WCHAR foldername[MAX_PATH],int counter);

	// Open a .binary file to save raw color streams
	void							OpenRawColorFile(WCHAR foldername[MAX_PATH],int counter);	
};

