//------------------------------------------------------------------------------
// <copyright file="NuiSkeletonStream.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include <map>
#include "NuiStream.h"
#include "NuiActivityWatcher.h"
#include "StreamSaver.h"

// Nui skeleton chooser mode
enum ChooserMode
{
    ChooserModeDefault,
    ChooserModeClosest1,
    ChooserModeClosest2,
    ChooserModeSticky1,
    ChooserModeSticky2,
    ChooserModeActive1,
    ChooserModeActive2
};

// Tracked player ID index
enum TrackIDIndex
{
    FirstTrackID = 0,
    SecondTrackID,
    TrackIDIndexCount
};

class NuiSkeletonStream : public NuiStream
{
public:
    /// <summary>
    /// Constructor
    /// <summary>
    /// <param name="pNuiSensor">The pointer to NUI sensor device instance</param>
    NuiSkeletonStream(INuiSensor* pNuiSensor);

    /// <summary>
    /// Destructor
    /// </summary>
   ~NuiSkeletonStream();

public:
    /// <summary>
    /// Start stream processing
    /// </summary>
    virtual HRESULT StartStream();

    /// <summary>
    /// Pause or resume stream processing
    /// </summary>
    /// <param name="pause">True to pause the stream and false to resume</param>
    virtual void PauseStream(bool pause);

    /// <summary>
    /// Check if incoming frame is ready
    /// </summary>
    void ProcessStreamFrame();

    /// <summary>
    /// Set near mode
    /// </summary>
    /// <param name="nearMode">True to enable near mode. False to disable</param>
    void SetNearMode(bool nearMode);

    /// <summary>
    /// Set seated mode
    /// </summary>
    /// <param name="seated">True to enable seated mode. False to disable</param>
    void SetSeatedMode(bool seated);

    /// <summary>
    /// Set chooser mode
    /// </summary>
    /// <param name="mode">Chooser mode to be set</param>
    void SetChooserMode(ChooserMode mode);

    /// <summary>
    /// Attach the second stream viewer to display skeleton
    /// </summary>
    /// <param name="pStreamViewer">The pointer to the stream viewer to be attached</param>
    void SetSecondStreamViewer(NuiStreamViewer* pViewer);

	NUI_SKELETON_FRAME*  nui_skeletonFrame;

private:
    /// <summary>
    /// Process on incoming frame
    /// <summary>
    void ProcessSkeleton();

    /// <summary>
    /// Update tracked skeletons according to current chooser mode
    /// </summary>
    void UpdateTrackedSkeletons();

    /// <summary>
    /// Find sticky skeletons to set tracked
    /// </summary>
    /// <param name="trackIDs">Array of skeleton tracking IDs</param>
    void ChooseStickySkeletons(DWORD trackIDs[TrackIDIndexCount]);

    /// <summary>
    /// Find closest skeleton to set tracked
    /// </summary>
    /// <param name="trackIDs">Array of skeleton tracking IDs</param>
    void ChooseClosestSkeletons(DWORD trackIDs[TrackIDIndexCount]);

    /// <summary>
    /// Find most active skeletons to set tracked
    /// </summary>
    /// <param name="trackIDs">Array of skeleton tracking IDs</param>
    void ChooseMostActiveSkeletons(DWORD trackIDs[TrackIDIndexCount]);

    /// <summary>
    /// Verify if stored tracked IDs are found in new skeleton frame
    /// </summary>
    /// <param name="trackIDs">Array of skeleton tracking IDs</param>
    void FindStickyIDs(DWORD trackIDs[TrackIDIndexCount]);

    /// <summary>
    /// Assign a new ID if old sticky is not found in new skeleton frame
    /// </summary>
    /// <param name="trackIDs">Array of skeleton tracking IDs</param>
    void AssignNewStickyIDs(DWORD trackIDs[TrackIDIndexCount]);

    /// <summary>
    /// Reset update flags for all activity watchers for further process
    /// </summary>
    void ResetActivityWatcherFlags();

    /// <summary>
    /// Update activity watcher items and their activity levels
    /// </summary>
    void UpdateActivityWatchers();

    /// <summary>
    /// Delete non-update activity watchers which has lost track to skeleton from stored collection
    /// </summary>
    void DeleteNonUpdateWatchers();

    /// <summary>
    /// Find most active IDs
    /// </summary>
    /// <param name="trackIDs">Array of skeleton tracking IDs</param>
    void FindMostActiveIDs(DWORD trackIDs[TrackIDIndexCount]);

    /// <summary>
    /// Assign the skeleton data to the stream viewers
    /// </summary>
    /// <param name="pFrame">The pointer to skeleton frame</param>
    void AssignSkeletonFrameToStreamViewers(const NUI_SKELETON_FRAME* pFrame);

private:
    bool                m_near;
    bool                m_seated;
    DWORD               m_stickyIDs[TrackIDIndexCount];
    ChooserMode         m_chooserMode;
    NUI_SKELETON_FRAME  m_skeletonFrame;
    NuiStreamViewer*    m_pSecondStreamViewer;

    std::map<int, NuiActivityWatcher*> m_activityWatchers;
};