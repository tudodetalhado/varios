/*
 *  kinectthread.cpp
 *  Kinect demo.
 *
 *  Kinect thread processing.
 *
 *  Created by radix on 06/30/11.
 *  Copyright Mykola Konyk, <mykola@konyk.org>, 2011.
 *
 *  This code is under Microsoft Reciprocal License (Ms-RL)
 *  Please see http://www.opensource.org/licenses/ms-rl.html
 *
 *  Important points about the license (from Ms-RL):
 *
 *  [A] For any file you distribute that contains code from the software (in source code or binary format), you must provide
 *  recipients the source code to that file along with a copy of this license, which license will govern that file.
 *  You may license other files that are entirely your own work and do not contain code from the software under any terms
 *  you choose.
 *
 *  [B] No Trademark License- This license does not grant you rights to use any contributors' name, logo, or trademarks.
 *
 *  [C] If you bring a patent claim against any contributor over patents that you claim are infringed by the software, your
 *  patent license from such contributor to the software ends automatically.
 *
 *  [D] If you distribute any portion of the software, you must retain all copyright, patent, trademark, and attribution notices
 *  that are present in the software.
 *
 *  [E] If you distribute any portion of the software in source code form, you may do so only under this license by including a
 *  complete copy of this license with your distribution. If you distribute any portion of the software in compiled or object
 *  code form, you may only do so under a license that complies with this license.
 *
 *  [F] The software is licensed "as-is." You bear the risk of using it. The contributors give no express warranties, guarantees
 *  or conditions. You may have additional consumer rights under your local laws which this license cannot change. To the extent
 *  permitted under your local laws, the contributors exclude the implied warranties of merchantability, fitness for a particular
 *  purpose and non-infringement.
 *
 */

#include <kinectthread.h>
#include <dialog.h>

namespace Kinect
{
	//--
	KinectThread::KinectThread(QObject* parent) :
	QThread(parent),
	m_running(false),
	m_color_next_frame_event(NULL),
	m_depth_next_frame_event(NULL),
	m_color_stream_handle(NULL),
	m_depth_stream_handle(NULL)
	{
		
	}


	//--
	KinectThread::~KinectThread()
	{
		
	}


	//--
	void
	KinectThread::run()
	{
		HANDLE events[2] = {m_color_next_frame_event, m_depth_next_frame_event};

		m_running = true;
		int event_idx;

		while(m_running)
		{
			// [rad] Wait for events, (not necessarily all) and time out is 100 msec.
			event_idx = WaitForMultipleObjects((sizeof(events) / sizeof(events[0])), events, FALSE, 100);

			// [rad] Process events.
			if(0 == event_idx)
			{
				// [rad] We have a color frame ready, emit signal.
				emit eventFrameColor();
			}
			else if(1 == event_idx)
			{
				// [rad] We have a depth frame ready, emit signal.
				emit eventFrameDepth();
			}

			usleep(15);
		}
	}


	//--
	void
	KinectThread::stop()
	{
		m_running = false;
	}


	//--
	void 
	KinectThread::setDepthHandles(HANDLE stream, HANDLE frame)
	{
		m_depth_next_frame_event = frame;
		m_depth_stream_handle = stream;
	}
	
	
	//--
	void 
	KinectThread::setColorHandles(HANDLE stream, HANDLE frame)
	{
		m_color_next_frame_event = frame;
		m_color_stream_handle = stream;
	}
}
