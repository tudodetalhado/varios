//------------------------------------------------------------------------------
// <copyright file="NuiDepthStream.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <cmath>
#include "NuiDepthStream.h"
#include "NuiStreamViewer.h"


/// <summary>
/// Constructor
/// <summary>
/// <param name="pNuiSensor">The pointer to NUI sensor device instance</param>
NuiDepthStream::NuiDepthStream(INuiSensor* pNuiSensor)
	: NuiStream(pNuiSensor)
	, m_imageType(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX)
	, m_nearMode(false)
	, m_depthTreatment(CLAMP_UNRELIABLE_DEPTHS)
	, nui_Depth(nullptr)
{
	
}

/// <summary>
/// Destructor
/// </summary>
NuiDepthStream::~NuiDepthStream()
{
}

/// <summary>
/// Attach viewer object to stream object
/// </summary>
/// <param name="pStreamViewer">The pointer to viewer object to attach</param>
/// <returns>Previously attached viewer object. If none, returns nullptr</returns>
NuiStreamViewer* NuiDepthStream::SetStreamViewer(NuiStreamViewer* pStreamViewer)
{
	if (pStreamViewer)
	{
		// Set image data to newly attached viewer object as well
		pStreamViewer->SetImage(&m_imageBuffer);
		pStreamViewer->SetImageType(m_imageType);
	}

	return NuiStream::SetStreamViewer(pStreamViewer);
}

/// <summary>
/// Set and reset near mode
/// </summary>
/// <param name="nearMode">True to enable near mode. False to disable</param>
void NuiDepthStream::SetNearMode(bool nearMode)
{
	m_nearMode = nearMode;
	if (INVALID_HANDLE_VALUE != m_hStreamHandle)
	{
		m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_hStreamHandle, (m_nearMode ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0));
	}
}

/// <summary>
/// Set depth treatment
/// </summary>
/// <param name="treatment">Depth treatment mode to set</param>
void NuiDepthStream::SetDepthTreatment(DEPTH_TREATMENT treatment)
{
	m_depthTreatment = treatment;
}

/// <summary>
/// Start stream processing.
/// </summary>
/// <returns>Indicate success or failure.</returns>
HRESULT NuiDepthStream::StartStream()
{
	return OpenStream(NUI_IMAGE_RESOLUTION_640x480); // Start stream with default resolution 640x480
}

/// <summary>
/// Open stream with a certain image resolution.
/// </summary>
/// <param name="resolution">Frame image resolution</param>
/// <returns>Indicates success or failure.</returns>
HRESULT NuiDepthStream::OpenStream(NUI_IMAGE_RESOLUTION resolution)
{
	m_imageType = HasSkeletalEngine(m_pNuiSensor) ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH;

	// Open depth stream
	HRESULT hr = m_pNuiSensor->NuiImageStreamOpen(m_imageType,
		resolution,
		0,
		2,
		GetFrameReadyEvent(),
		&m_hStreamHandle);
	if (SUCCEEDED(hr))
	{
		m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_hStreamHandle, m_nearMode ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0);   // Set image flags
		m_imageBuffer.SetImageSize(resolution); // Set source image resolution to image buffer
	}

	return hr;
}

/// <summary>
/// Process an incoming stream frame
/// </summary>
void NuiDepthStream::ProcessStreamFrame()
{
	if (WAIT_OBJECT_0 == WaitForSingleObject(GetFrameReadyEvent(), 0))
	{
		// if we have received any valid new depth data we may need to draw
		ProcessDepth();
	}
}

/// <summary>
/// Retrieve depth data from stream frame
/// </summary>
void NuiDepthStream::ProcessDepth()
{
	HRESULT         hr;
	NUI_IMAGE_FRAME imageFrame;

	// Attempt to get the depth frame
	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_hStreamHandle, 0, &imageFrame);
	if (FAILED(hr))
	{
		return;
	}

	if (m_paused)
	{
		// Stream paused. Skip frame process and release the frame.
		goto ReleaseFrame;
	}

	BOOL nearMode;
	INuiFrameTexture* pTexture;

	// Get the depth image pixel texture
	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_hStreamHandle, &imageFrame, &nearMode, &pTexture); 
	if (FAILED(hr))
	{
		goto ReleaseFrame;
	}

	NUI_LOCKED_RECT lockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	pTexture->LockRect(0, &lockedRect, NULL, 0);

	// Make sure we've received valid data
	if (lockedRect.Pitch != 0)
	{

		// Conver depth data to color image and copy to image buffer
		m_imageBuffer.CopyDepth(lockedRect.pBits, lockedRect.size, nearMode, m_depthTreatment);		

		// save the Depth stream as image
		nui_Depth = &m_imageBuffer;	
		DepthFrameNumber = imageFrame.dwFrameNumber;
		DepthliTimeStamp = imageFrame.liTimeStamp;

		// save the Depth stream as raw data
		{
			// Initialize pixel pointers to start and end of image buffer
			NUI_DEPTH_IMAGE_PIXEL* pPixelRun = (NUI_DEPTH_IMAGE_PIXEL*)lockedRect.pBits;
			NUI_DEPTH_IMAGE_PIXEL* pPixelEnd = pPixelRun + m_imageBuffer.GetWidth()*m_imageBuffer.GetHeight(); 

			int i = 0;
			// Run through pixels
			while (pPixelRun < pPixelEnd)
			{
				// Get pixel depth and player index 
				pBuffer[i] = pPixelRun->depth;

				// Move the pointers to next pixel
				++i;
				++pPixelRun;
			}
		}

		if (EnHideView)
		{
			m_pStreamViewer->HideView();
			//goto ReleaseFrame;
		}else if (m_pStreamViewer) // Draw ou the data with Direct2D
		{
			m_pStreamViewer->ShowView();
			m_pStreamViewer->SetImage(&m_imageBuffer);
		}

	}

	// Done with the texture. Unlock and release it
	pTexture->UnlockRect(0);
	pTexture->Release();

ReleaseFrame:
	// Release the frame
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_hStreamHandle, &imageFrame);
}