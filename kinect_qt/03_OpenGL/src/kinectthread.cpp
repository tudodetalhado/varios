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
