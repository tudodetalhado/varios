//------------------------------------------------------------------------------
// <copyright file="NuiDepthStream.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "NuiStream.h"
#include "NuiImageBuffer.h"
#include "StreamSaver.h"

class NuiDepthStream : public NuiStream
{
public:
    /// <summary>
    /// Constructor
    /// <summary>
    /// <param name="pNuiSensor">The pointer to NUI sensor device instance</param>
    NuiDepthStream(INuiSensor* pNuiSensor);

    /// <summary>
    /// Destructor
    /// </summary>
   ~NuiDepthStream();

public:
    /// <summary>
    /// Attach viewer object to stream object
    /// </summary>
    /// <param name="pStreamViewer">The pointer to viewer object to attach</param>
    /// <returns>Previously attached viewer object. If none, returns nullptr</returns>
    virtual NuiStreamViewer* SetStreamViewer(NuiStreamViewer* pStreamViewer);

    /// <summary>
    /// Start stream processing.
    /// </summary>
    /// <returns>Indicate success or failure.</returns>
    virtual HRESULT StartStream();

    /// <summary>
    /// Open stream with a certain image resolution.
    /// </summary>
    /// <param name="resolution">Frame image resolution</param>
    /// <returns>Indicates success or failure.</returns>
    HRESULT OpenStream(NUI_IMAGE_RESOLUTION resolution);

    /// <summary>
    /// Process an incoming stream frame
    /// </summary>
    virtual void ProcessStreamFrame();

    /// <summary>
    /// Set and reset near mode
    /// </summary>
    /// <param name="nearMode">True to enable near mode. False to disable</param>
    void SetNearMode(bool nearMode);

    /// <summary>
    /// Set depth treatment
    /// </summary>
    /// <param name="treatment">Depth treatment mode to set</param>
    void SetDepthTreatment(DEPTH_TREATMENT treatment);

	NuiImageBuffer*		nui_Depth;
	USHORT				pBuffer[640*480];
	DWORD				DepthFrameNumber;
	LARGE_INTEGER		DepthliTimeStamp;
	bool				EnHideView;

private:
    /// <summary>
    /// Retrieve depth data from stream frame
    /// </summary>
    void ProcessDepth();

private:
    bool            m_nearMode;
    NUI_IMAGE_TYPE  m_imageType;
    NuiImageBuffer  m_imageBuffer;
    DEPTH_TREATMENT m_depthTreatment;	
};