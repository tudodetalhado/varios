//------------------------------------------------------------------------------
// <copyright file="NuiColorStream.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include "NuiColorStream.h"
#include "NuiStreamViewer.h"


#define Color		  1
#define STREAM        1
#define BG			  2


/// <summary>
/// Constructor
/// </summary>
/// <param name="pNuiSensor">The pointer to Nui sensor object</param>
NuiColorStream::NuiColorStream(INuiSensor* pNuiSensor)
	: NuiStream(pNuiSensor)
	, m_imageType(NUI_IMAGE_TYPE_COLOR)
	, m_imageResolution(NUI_IMAGE_RESOLUTION_640x480)
	, nui_Color(nullptr)
{
	
}

/// <summary>
/// Destructor
/// </summary>
NuiColorStream::~NuiColorStream()
{
}

/// <summary>
/// Attach viewer object to stream object
/// </summary>
/// <param name="pStreamViewer">The pointer to viewer object to attach</param>
/// <returns>Previously attached viewer object. If none, returns nullptr</returns>
NuiStreamViewer* NuiColorStream::SetStreamViewer(NuiStreamViewer* pStreamViewer)
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
/// Start stream processing.
/// </summary>
/// <returns>Indicate success or failure.</returns>
HRESULT NuiColorStream::StartStream()
{
	SetImageType(NUI_IMAGE_TYPE_COLOR);                 // Set default image type to color image
	SetImageResolution(NUI_IMAGE_RESOLUTION_640x480);   // Set default image resolution to 640x480
	return OpenStream();
}

/// <summary>
/// Open stream with a certain image resolution.
/// </summary>
/// <returns>Indicates success or failure.</returns>
HRESULT NuiColorStream::OpenStream()
{
	// Open color stream.
	HRESULT hr = m_pNuiSensor->NuiImageStreamOpen(m_imageType,
		m_imageResolution,
		0,
		2,
		GetFrameReadyEvent(),
		&m_hStreamHandle);

	// Reset image buffer
	if (SUCCEEDED(hr))
	{
		m_imageBuffer.SetImageSize(m_imageResolution);  // Set source image resolution to image buffer
	}

	return hr;
}

/// <summary>
/// Set image type. Only color image types are acceptable
/// </summary>
/// <param name="type">Image type to be set</param>
void NuiColorStream::SetImageType(NUI_IMAGE_TYPE type)
{
	switch (type)
	{
	case NUI_IMAGE_TYPE_COLOR:
	case NUI_IMAGE_TYPE_COLOR_YUV:
	case NUI_IMAGE_TYPE_COLOR_INFRARED:
	case NUI_IMAGE_TYPE_COLOR_RAW_BAYER:
		m_imageType   = type;
		break;

	default:
		break;
	}

	// Sync the stream viewer image type
	m_pStreamViewer->SetImageType(m_imageType);
}

/// <summary>
/// Set image resolution. Only 640x480 and 1280x960 formats are acceptable
/// </summary>
/// <param name="resolution">Image resolution to be set</param>
void NuiColorStream::SetImageResolution(NUI_IMAGE_RESOLUTION resolution)
{
	switch (resolution)
	{
	case NUI_IMAGE_RESOLUTION_640x480:
	case NUI_IMAGE_RESOLUTION_1280x960:
		m_imageResolution = resolution;
		break;

	default:
		break;
	}
}

/// <summary>
/// Process a incoming stream frame
/// </summary>
void NuiColorStream::ProcessStreamFrame()
{
	if (WAIT_OBJECT_0 == WaitForSingleObject(GetFrameReadyEvent(), 0))
	{
		// Frame ready event has been set. Proceed to process incoming frame
		ProcessColor();
	}
}

/// <summary>
/// Process the incoming color frame
/// </summary>
void NuiColorStream::ProcessColor()
{
	HRESULT hr;
	NUI_IMAGE_FRAME imageFrame;

	// Attempt to get the color frame
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

	INuiFrameTexture* pTexture = imageFrame.pFrameTexture;

	// Lock the frame data so the Kinect knows not to modify it while we are reading it
	NUI_LOCKED_RECT lockedRect;
	pTexture->LockRect(0, &lockedRect, NULL, 0);

	// Make sure we've received valid data
	if (lockedRect.Pitch != 0)
	{
		switch (m_imageType)
		{
		case NUI_IMAGE_TYPE_COLOR_RAW_BAYER:    // Convert raw bayer data to color image and copy to image buffer
			m_imageBuffer.CopyBayer(lockedRect.pBits, lockedRect.size);
			break;

		case NUI_IMAGE_TYPE_COLOR_INFRARED:     // Convert infrared data to color image and copy to image buffer
			m_imageBuffer.CopyInfrared(lockedRect.pBits, lockedRect.size);
			break;

		default:    // Copy color data to image buffer
			m_imageBuffer.CopyRGB(lockedRect.pBits, lockedRect.size);
			break;
		}

		// save the color stream
		nui_Color = &m_imageBuffer;	
		ColorFrameNumber = imageFrame.dwFrameNumber;
		ColorliTimeStamp = imageFrame.liTimeStamp;

		// hide the view
		if (EnHideView)
		{
			m_pStreamViewer->HideView();
		} else if (m_pStreamViewer) // Draw ou the data with Direct2D
		{
			m_pStreamViewer->ShowView();
			m_pStreamViewer->SetImage(&m_imageBuffer); // Set image data to viewer
		}
	}

	// Unlock frame data
	pTexture->UnlockRect(0);

ReleaseFrame:
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_hStreamHandle, &imageFrame);
}

