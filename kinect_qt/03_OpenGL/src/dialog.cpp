#include <dialog.h>
#include <kinectthread.h>
#include <display.h>

#include <ui_kinect.h>

#include <QtGui/QMessageBox>

namespace Kinect
{
	//--
	Dialog::Dialog() :
	QDialog(0, Qt::Dialog | Qt::WindowMinimizeButtonHint),
	m_thread(NULL),
	m_display(NULL),
	m_ui(new Ui::Dialog()),
	m_kinect_initialized(false),
	m_kinect_angle(0),
	m_color_next_frame_event(NULL),
	m_depth_next_frame_event(NULL),
	m_color_stream_handle(NULL),
	m_depth_stream_handle(NULL)
	{
		// [rad] Setup ui for this dialog.
		setupUi();

		setWindowTitle(tr("Kinect Demo 03: OpenGL"));

		// [rad] Disable resizing.
		setSizeGripEnabled(false);
		setFixedSize(width(), height());

		// [rad] Initialize kinect.
		kinectInitialize();
	}


	//--
	Dialog::~Dialog()
	{
		// [rad] Shutdown kinect.
		kinectFinalize();

		if(m_ui)
		{
			delete(m_ui);
		}
	}


	//--
	void
	Dialog::setupUi()
	{
		// [rad] Construct internal Qt dialog ui.
		m_ui->setupUi(this);

		// [rad] Connect callbacks and setup gui elements.
		QObject::connect(m_ui->button_up, SIGNAL(clicked()), this, SLOT(eventButtonUp()));
		QObject::connect(m_ui->button_down, SIGNAL(clicked()), this, SLOT(eventButtonDown()));
		QObject::connect(m_ui->button_center, SIGNAL(clicked()), this, SLOT(eventButtonCenter()));

		// [rad] Construct GL renderer.
		m_display = m_ui->camera;
	}


	//--
	void
	Dialog::kinectInitialize()
	{
		if(m_kinect_initialized)
		{
			return;
		}

		// [rad] We don't need any of the advaced features.
		if(S_OK != ::NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH))
		{
			// [rad] Error initializing.
			QMessageBox msg_box;
			
			msg_box.setText("Nui Error: Can't initialize.");
			msg_box.exec();

			return;
		}

		// [rad] Create kinect frame events.
		m_color_next_frame_event = ::CreateEvent(NULL, TRUE, FALSE, NULL);
    	m_depth_next_frame_event = ::CreateEvent(NULL, TRUE, FALSE, NULL);

    	// [rad] Disable skeleton tracking in this demo.
    	//::NuiSkeletonTrackingDisable();

    	{
    		// [rad] Attempt to open color stream reader.
    		HRESULT hr = ::NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, m_color_next_frame_event, &m_color_stream_handle);
    	
	    	if(FAILED(hr))
	    	{
	    		// [rad] Error opening color image stream.
				QMessageBox msg_box;
				
				msg_box.setText("Nui Error: Unable to open Color image stream.");
				msg_box.exec();

	    		return;
	    	}

	    	// [rad] Create display for this stream.
	    	//m_display_color = new Display(m_ui->camera_color, 640, 480, this);
	    }

	    {
	    	
    		HRESULT hr = ::NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480, 0, 2, m_depth_next_frame_event, &m_depth_stream_handle);
    	
	    	if(FAILED(hr))
	    	{
	    		// [rad] Error opening depth image stream.
				QMessageBox msg_box;
				
				msg_box.setText("Nui Error: Unable to open Depth image stream.");
				msg_box.exec();

	    		return;
	    	}

	    	// [rad] Create display for this stream.
	    	//m_display_depth = new Display(m_ui->camera_depth, 640, 480, this);
	    }

    	// [rad] Create kinect processing thread.
    	m_thread = new KinectThread(this);
    	
    	// [rad] Connect kinect thread events.
    	QObject::connect(m_thread, SIGNAL(eventFrameDepth()), this, SLOT(eventFrameDepth()));
    	QObject::connect(m_thread, SIGNAL(eventFrameColor()), this, SLOT(eventFrameColor()));

    	// [rad] Set handles for thread to process.
    	m_thread->setDepthHandles(m_depth_stream_handle, m_depth_next_frame_event);
    	m_thread->setColorHandles(m_color_stream_handle, m_color_next_frame_event);

    	// [rad] Start kinect thread.
    	m_thread->start();

    	// [rad] Kinect was initialized.
		m_kinect_initialized = true;

    	// [rad] Center camera.
		eventButtonCenter();
	}


	//--
	void
	Dialog::kinectFinalize()
	{
		if(m_kinect_initialized)
		{
			// [rad] Stop processing thread.
			m_thread->stop();

			// [rad] Call library finalization.
			::NuiShutdown();	

			// [rad] Delete kinect frame events.
			if(m_depth_next_frame_event && (INVALID_HANDLE_VALUE != m_depth_next_frame_event))
			{
				::CloseHandle(m_depth_next_frame_event);
				m_depth_next_frame_event = NULL;
			}

			if(m_color_next_frame_event && (INVALID_HANDLE_VALUE != m_color_next_frame_event))
			{
				::CloseHandle(m_color_next_frame_event);
				m_color_next_frame_event = NULL;
			}

			// [rad] Delete processing thread.
			//delete(m_thread);

			// [rad] Delete displays.
			//delete(m_display_depth);
			//delete(m_display_color);
		}
	}


	//--
	void
	Dialog::eventFrameDepth()
	{
		const NUI_IMAGE_FRAME* image_frame = NULL;

		HRESULT hr = ::NuiImageStreamGetNextFrame(m_depth_stream_handle, 0, &image_frame);
		
		if(FAILED(hr))
		{
			return;	
		}

		NuiImageBuffer* texture = image_frame->pFrameTexture;
		KINECT_LOCKED_RECT locked_rect;
		texture->LockRect(0, &locked_rect, NULL, 0);

		if(locked_rect.Pitch != 0)
    	{
    		// [rad] Do nothing.
    	}

    	texture->UnlockRect(0);

		::NuiImageStreamReleaseFrame(m_depth_stream_handle, image_frame);
	}


	//--
	void
	Dialog::eventFrameColor()
	{
		const NUI_IMAGE_FRAME* image_frame = NULL;

		HRESULT hr = ::NuiImageStreamGetNextFrame(m_color_stream_handle, 0, &image_frame);
		
		if(FAILED(hr))
		{
			return;	
		}

		NuiImageBuffer* texture = image_frame->pFrameTexture;
		KINECT_LOCKED_RECT locked_rect;
		texture->LockRect(0, &locked_rect, NULL, 0);

		if(locked_rect.Pitch != 0)
    	{
    		m_display->drawColor((const unsigned char*) locked_rect.pBits);
    	}

    	texture->UnlockRect(0);

		::NuiImageStreamReleaseFrame(m_color_stream_handle, image_frame);
	}


	//--
	void
	Dialog::eventButtonCenter()
	{
		if(m_kinect_initialized)
		{
			m_kinect_angle = 0;
			::NuiCameraElevationSetAngle(m_kinect_angle);
		}
	}


	//--
	void
	Dialog::eventButtonUp()
	{
		if(m_kinect_initialized)
		{
			// [rad] Check so that we don't go over max elevation.
			if(m_kinect_angle + 5 > NUI_CAMERA_ELEVATION_MAXIMUM)
			{
				return;
			}

			m_kinect_angle += 5;
			::NuiCameraElevationSetAngle(m_kinect_angle);
		}
	}


	//--
	void
	Dialog::eventButtonDown()
	{
		if(m_kinect_initialized)
		{
			// [rad] Check so that we don't go over max elevation.
			if(m_kinect_angle - 5 < NUI_CAMERA_ELEVATION_MINIMUM)
			{
				return;
			}

			m_kinect_angle -= 5;
			::NuiCameraElevationSetAngle(m_kinect_angle);
		}
	}
}
