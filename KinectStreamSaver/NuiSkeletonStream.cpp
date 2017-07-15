//------------------------------------------------------------------------------
// <copyright file="NuiSkeletonStream.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include "NuiSkeletonStream.h"
#include "NuiStreamViewer.h"


/// <summary>
/// Constructor
/// <summary>
/// <param name="pNuiSensor">The pointer to NUI sensor device instance</param>
NuiSkeletonStream::NuiSkeletonStream(INuiSensor* pNuiSensor)
    : NuiStream(pNuiSensor)
    , m_near(false)
    , m_seated(false)
    , m_chooserMode(ChooserModeDefault)
    , m_pSecondStreamViewer(nullptr)
	, nui_skeletonFrame(nullptr)
{
    m_stickyIDs[FirstTrackID] = 0;
    m_stickyIDs[SecondTrackID] = 0;
	nui_skeletonFrame = NULL;
}

/// <summary>
/// Destructor
/// </summary>
NuiSkeletonStream::~NuiSkeletonStream()
{
    // Clear activity watchers
    for (auto itr = m_activityWatchers.begin(); itr != m_activityWatchers.end(); ++itr)
    {
        delete itr->second;
    }
    m_activityWatchers.clear();
}

/// <summary>
/// Set near mode
/// </summary>
/// <param name="nearMode">True to enable near mode. False to disable</param>
void NuiSkeletonStream::SetNearMode(bool nearMode)
{
    if (m_near != nearMode)
    {
        m_near = nearMode;
        StartStream();  // Restart stream with new parameter value
    }
}

/// <summary>
/// Set seated mode
/// </summary>
/// <param name="seated">True to enable seated mode. False to disable</param>
void NuiSkeletonStream::SetSeatedMode(bool seated)
{
    if (m_seated != seated)
    {
        m_seated = seated;
        StartStream();  // Restart stream with new parameter value
    }
}

/// <summary>
/// Set chooser mode
/// </summary>
/// <param name="mode">Chooser mode to be set</param>
void NuiSkeletonStream::SetChooserMode(ChooserMode mode)
{
    if (m_chooserMode != mode)
    {
        m_chooserMode = mode;
        StartStream();  // Restart stream with new parameter value
    }
}

/// <summary>
/// Attach the second stream viewer to display skeleton
/// </summary>
/// <param name="pStreamViewer">The pointer to the stream viewer to be attached</param>
void NuiSkeletonStream::SetSecondStreamViewer(NuiStreamViewer* pStreamViewer)
{
    m_pSecondStreamViewer = pStreamViewer;
}

/// <summary>
/// Start stream processing
/// </summary>
HRESULT NuiSkeletonStream::StartStream()
{
    if (HasSkeletalEngine(m_pNuiSensor))
    {
        if (m_paused)
        {
            // Clear skeleton data in stream viewers
            AssignSkeletonFrameToStreamViewers(nullptr);

            // Disable tracking skeleton
            return m_pNuiSensor->NuiSkeletonTrackingDisable();
        }
        else
        {
            // Enable tracking skeleton
            DWORD flags = (m_seated ? NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT : 0) | (m_near ? NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE : 0)
                | (ChooserModeDefault != m_chooserMode ? NUI_SKELETON_TRACKING_FLAG_TITLE_SETS_TRACKED_SKELETONS : 0);
            return m_pNuiSensor->NuiSkeletonTrackingEnable(GetFrameReadyEvent(), flags);
        }
    }

    return E_FAIL;
}

/// <summary>
/// Pause or resume stream processing
/// </summary>
/// <param name="pause">True to pause the stream and false to resume</param>
void NuiSkeletonStream::PauseStream(bool pause)
{
    if (m_paused != pause)
    {
        m_paused = pause;
        StartStream();
    }
}

/// <summary>
/// Check if incoming frame is ready
/// </summary>
void NuiSkeletonStream::ProcessStreamFrame()
{
    if (WAIT_OBJECT_0 == WaitForSingleObject(GetFrameReadyEvent(), 0))
    {
        // if we have received any valid new depth data we may need to draw
        ProcessSkeleton();
    }
}

/// <summary>
/// Process on incoming frame
/// <summary>
void NuiSkeletonStream::ProcessSkeleton()
{
    // Retrieve skeleton frame
    HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame(0, &m_skeletonFrame);
    if (FAILED(hr) || m_paused)
    {
        // If occur error when get skeleton data or pause tracking skeleton,
        // clear skeleton data in stream viewers
        AssignSkeletonFrameToStreamViewers(nullptr);
        return;
    }

    // smooth out the skeleton data
    m_pNuiSensor->NuiTransformSmooth(&m_skeletonFrame, nullptr);

    // Set skeleton data to stream viewers
    AssignSkeletonFrameToStreamViewers(&m_skeletonFrame);

	// save the Skeleton stream
	nui_skeletonFrame = &m_skeletonFrame;

    UpdateTrackedSkeletons();
}

/// <summary>
/// Update tracked skeletons according to current chooser mode
/// </summary>
void NuiSkeletonStream::UpdateTrackedSkeletons()
{
    DWORD trackIDs[TrackIDIndexCount] = {0};

    if (ChooserModeClosest1 == m_chooserMode || ChooserModeClosest2 == m_chooserMode)
    {
        ChooseClosestSkeletons(trackIDs);
    }
    else if (ChooserModeSticky1 == m_chooserMode || ChooserModeSticky2 == m_chooserMode)
    {
        ChooseStickySkeletons(trackIDs);
    }
    else if (ChooserModeActive1 == m_chooserMode || ChooserModeActive2 == m_chooserMode)
    {
        ChooseMostActiveSkeletons(trackIDs);
    }

    if (ChooserModeClosest1 == m_chooserMode || ChooserModeSticky1 == m_chooserMode || ChooserModeActive1 == m_chooserMode)
    {
        // Track only one player ID. The second ID is not used
        trackIDs[SecondTrackID] = 0;
    }

    m_pNuiSensor->NuiSkeletonSetTrackedSkeletons(trackIDs);
}

/// <summary>
/// Find closest skeleton to set tracked
/// </summary>
/// <param name="trackIDs">Array of skeleton tracking IDs</param>
void NuiSkeletonStream::ChooseClosestSkeletons(DWORD trackIDs[TrackIDIndexCount])
{
    ZeroMemory(trackIDs, TrackIDIndexCount * sizeof(DWORD));

    // Initial depth array with max posible value
    USHORT nearestDepth[TrackIDIndexCount] = {NUI_IMAGE_DEPTH_MAXIMUM, NUI_IMAGE_DEPTH_MAXIMUM};

    for (int i = 0; i < NUI_SKELETON_COUNT; i++)
    {
        if (NUI_SKELETON_NOT_TRACKED != m_skeletonFrame.SkeletonData[i].eTrackingState)
        {
            LONG   x, y;
            USHORT depth;

            // Transform skeleton coordinates to depth image
            NuiTransformSkeletonToDepthImage(m_skeletonFrame.SkeletonData[i].Position, &x, &y, &depth);

            // Compare depth to peviously found item
            if (depth < nearestDepth[FirstTrackID])
            {
                // Move depth and track ID in first place to second place and assign with the new closer one
                nearestDepth[SecondTrackID] = nearestDepth[FirstTrackID];
                nearestDepth[FirstTrackID]  = depth;

                trackIDs[SecondTrackID] = trackIDs[FirstTrackID];
                trackIDs[FirstTrackID]  = m_skeletonFrame.SkeletonData[i].dwTrackingID;
            }
            else if (depth < nearestDepth[SecondTrackID])
            {
                // Replace old depth and track ID in second place with the newly found closer one
                nearestDepth[SecondTrackID] = depth;
                trackIDs[SecondTrackID]     = m_skeletonFrame.SkeletonData[i].dwTrackingID;
            }
        }
    }
}

/// <summary>
/// Find sticky skeletons to set tracked
/// </summary>
/// <param name="trackIDs">Array of skeleton tracking IDs</param>
void NuiSkeletonStream::ChooseStickySkeletons(DWORD trackIDs[TrackIDIndexCount])
{
    ZeroMemory(trackIDs, TrackIDIndexCount * sizeof(DWORD));

    FindStickyIDs(trackIDs);
    AssignNewStickyIDs(trackIDs);

    // Update stored sticky IDs
    m_stickyIDs[FirstTrackID]  = trackIDs[FirstTrackID];
    m_stickyIDs[SecondTrackID] = trackIDs[SecondTrackID];
}

/// <summary>
/// Verify if stored tracked IDs are found in new skeleton frame
/// </summary>
/// <param name="trackIDs">Array of skeleton tracking IDs</param>
void NuiSkeletonStream::FindStickyIDs(DWORD trackIDs[TrackIDIndexCount])
{
    for(int i = 0; i < TrackIDIndexCount; i++)
    {
        for(int j = 0; j < NUI_SKELETON_COUNT; j++)
        {
            if(NUI_SKELETON_NOT_TRACKED != m_skeletonFrame.SkeletonData[j].eTrackingState)
            {
                DWORD trackID = m_skeletonFrame.SkeletonData[j].dwTrackingID;
                if(trackID == m_stickyIDs[i])
                {
                    trackIDs[i] = trackID;
                    break;
                }
            }
        }
    }
}

/// <summary>
/// Assign a new ID if old sticky is not found in new skeleton frame
/// </summary>
/// <param name="trackIDs">Array of skeleton tracking IDs</param>
void NuiSkeletonStream::AssignNewStickyIDs(DWORD trackIDs[TrackIDIndexCount])
{
    for (int i = 0; i < NUI_SKELETON_COUNT; i++)
    {
        if (trackIDs[FirstTrackID] && trackIDs[SecondTrackID])
        {
            break;
        }

        if (NUI_SKELETON_NOT_TRACKED != m_skeletonFrame.SkeletonData[i].eTrackingState)
        {
            DWORD trackID = m_skeletonFrame.SkeletonData[i].dwTrackingID;

            if (!trackIDs[FirstTrackID] && trackID != trackIDs[SecondTrackID])
            {
                trackIDs[FirstTrackID] = trackID;
            }
            else if (!trackIDs[SecondTrackID] && trackID != trackIDs[FirstTrackID])
            {
                trackIDs[SecondTrackID] = trackID;
            }
        }
    }
}

/// <summary>
/// Find most active skeletons to set tracked
/// </summary>
/// <param name="trackIDs">Array of skeleton tracking IDs</param>
void NuiSkeletonStream::ChooseMostActiveSkeletons(DWORD trackIDs[TrackIDIndexCount])
{
    ZeroMemory(trackIDs, TrackIDIndexCount * sizeof(DWORD));

    // Clear update flag for all active watchers. Delete not updated active watchers later because IDs are not tracked in this pass.
    ResetActivityWatcherFlags();

    // Update watchers's activity levels and update flags
    UpdateActivityWatchers();

    // Delete not updated activity watchers because we've lost the track ID in this pass
    DeleteNonUpdateWatchers();

    // Find out highest activity level IDs
    FindMostActiveIDs(trackIDs);
}

//// <summary>
/// Reset update flags for all activity watchers for further process
/// </summary>
void NuiSkeletonStream::ResetActivityWatcherFlags()
{
    for (auto itr = m_activityWatchers.begin(); itr != m_activityWatchers.end(); ++itr)
    {
        itr->second->SetUpdateFlag(false);
    }
}

/// <summary>
/// Update activity watcher items and their activity levels
/// </summary>
void NuiSkeletonStream::UpdateActivityWatchers()
{
    for (int i = 0; i < NUI_SKELETON_COUNT; i++)
    {
        if (NUI_SKELETON_NOT_TRACKED != m_skeletonFrame.SkeletonData[i].eTrackingState)
        {
            DWORD id  = m_skeletonFrame.SkeletonData[i].dwTrackingID;
            auto  itr = m_activityWatchers.find(id);

            if (m_activityWatchers.end() != itr)
            {
                // Activity watcher related to this ID is found. Update its activity level
                itr->second->UpdateActivity(m_skeletonFrame.SkeletonData[i]);
                itr->second->SetUpdateFlag(true);
            }
            else
            {
                // No activity watcher related to this ID is found. Create a new one for it
                NuiActivityWatcher* pWatcher = new NuiActivityWatcher(m_skeletonFrame.SkeletonData[i]);
                pWatcher->SetUpdateFlag(true);

                m_activityWatchers.insert(std::make_pair(id, pWatcher));
            }
        }
    }
}

/// <summary>
/// Delete non-update activity watchers which has lost track to skeleton from stored collection
/// </summary>
void NuiSkeletonStream::DeleteNonUpdateWatchers()
{
    for (auto itr = m_activityWatchers.begin(); itr != m_activityWatchers.end();)
    {
        if (itr->second->GetUpdateFlag())
        {
            ++itr;
        }
        else
        {
            delete itr->second;
            itr = m_activityWatchers.erase(itr);
        }
    }
}

/// <summary>
/// Find most active IDs
/// </summary>
/// <param name="trackIDs">Array of skeleton tracking IDs</param>
void NuiSkeletonStream::FindMostActiveIDs(DWORD trackIDs[TrackIDIndexCount])
{
    // Initialize activity levels with negtive valus so they can replaced by any found activity levels
    FLOAT activityLevels[TrackIDIndexCount] = {-1.0f, -1.0f};

    // Run through activity watchers
    for (auto itr = m_activityWatchers.begin(); itr != m_activityWatchers.end(); ++itr)
    {
        // Get calculated activity level
        FLOAT level = itr->second->GetActivityLevel();

        // Compare to previously found activity levels
        if (level > activityLevels[FirstTrackID])
        {
            // Move first track ID and activity level to second place. Assign newly found higher activity level and ID to first place
            activityLevels[SecondTrackID] = activityLevels[FirstTrackID];
            activityLevels[FirstTrackID]  = level;

            trackIDs[SecondTrackID]       = trackIDs[FirstTrackID];
            trackIDs[FirstTrackID]        = itr->first;
        }
        else if (level > activityLevels[SecondTrackID])
        {
            // Replace the previous one
            activityLevels[SecondTrackID] = level;
            trackIDs[SecondTrackID]       = itr->first;
        }
    }
}

/// <summary>
/// Assign the skeleton data to the stream viewers
/// </summary>
/// <param name="pFrame">The pointer to skeleton frame</param>
void NuiSkeletonStream::AssignSkeletonFrameToStreamViewers(const NUI_SKELETON_FRAME* pFrame)
{
    if (m_pStreamViewer)
    {
        m_pStreamViewer->SetSkeleton(pFrame);
    }

    if (m_pSecondStreamViewer)
    {
        m_pSecondStreamViewer->SetSkeleton(pFrame);
    }
}