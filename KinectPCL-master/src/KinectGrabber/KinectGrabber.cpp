#include "KinectGrabber.h"

//#ifdef _DEBUG
	#define _SCL_SECURE_NO_WARNINGS
//#endif

namespace pcl
{


	pcl::KinectGrabber::KinectGrabber()
		: sensor(nullptr)
		, mapper(nullptr)
		, colorSource(nullptr)
		, colorReader(nullptr)
		, depthSource(nullptr)
		, depthReader(nullptr)
		, result(S_OK)
		, colorWidth(1920)
		, colorHeight(1080)
		, colorBuffer()
		, depthWidth(512)
		, depthHeight(424)
		, depthBuffer()
		, running(false)
		, quit(false)
		, signal_PointXYZ(nullptr)
		, signal_PointXYZRGB(nullptr)
		, signal_FrameBuffer(nullptr)
	{
		// Create Sensor Instance
		result = GetDefaultKinectSensor(&sensor);
		if (FAILED(result)){
			throw std::exception("Exception : GetDefaultKinectSensor()");
		}

		// Open Sensor
		result = sensor->Open();
		if (FAILED(result)){
			throw std::exception("Exception : IKinectSensor::Open()");
		}

		// Retrieved Coordinate Mapper
		result = sensor->get_CoordinateMapper(&mapper);
		if (FAILED(result)){
			throw std::exception("Exception : IKinectSensor::get_CoordinateMapper()");
		}

		// Retrieved Color Frame Source
		result = sensor->get_ColorFrameSource(&colorSource);
		if (FAILED(result)){
			throw std::exception("Exception : IKinectSensor::get_ColorFrameSource()");
		}

		// Retrieved Depth Frame Source
		result = sensor->get_DepthFrameSource(&depthSource);
		if (FAILED(result)){
			throw std::exception("Exception : IKinectSensor::get_DepthFrameSource()");
		}

		// Retrieved Color Frame Size
		IFrameDescription* colorDescription;
		result = colorSource->get_FrameDescription(&colorDescription);
		if (FAILED(result)){
			throw std::exception("Exception : IColorFrameSource::get_FrameDescription()");
		}

		result = colorDescription->get_Width(&colorWidth); // 1920
		if (FAILED(result)){
			throw std::exception("Exception : IFrameDescription::get_Width()");
		}

		result = colorDescription->get_Height(&colorHeight); // 1080
		if (FAILED(result)){
			throw std::exception("Exception : IFrameDescription::get_Height()");
		}

		SafeRelease(colorDescription);

		// To Reserve Color Frame Buffer
		colorBuffer.resize(colorWidth * colorHeight * 4);

		// Retrieved Depth Frame Size
		IFrameDescription* depthDescription;
		result = depthSource->get_FrameDescription(&depthDescription);
		if (FAILED(result)){
			throw std::exception("Exception : IDepthFrameSource::get_FrameDescription()");
		}

		result = depthDescription->get_Width(&depthWidth); // 512
		if (FAILED(result)){
			throw std::exception("Exception : IFrameDescription::get_Width()");
		}

		result = depthDescription->get_Height(&depthHeight); // 424
		if (FAILED(result)){
			throw std::exception("Exception : IFrameDescription::get_Height()");
		}
		
		SafeRelease(depthDescription);

		// To Reserve Depth Frame Buffer
		depthBuffer.resize(depthWidth * depthHeight);

		signal_PointXYZ = createSignal<signal_Kinect2_PointXYZ>();
		signal_PointXYZRGB = createSignal<signal_Kinect2_PointXYZRGB>();
		signal_FrameBuffer = createSignal<signal_Kinect2_FrameBuffer>();
	}

	pcl::KinectGrabber::~KinectGrabber() throw()
	{
		stop();

		disconnect_all_slots<signal_Kinect2_PointXYZ>();
		disconnect_all_slots<signal_Kinect2_PointXYZRGB>();
		disconnect_all_slots<signal_Kinect2_FrameBuffer>();

		// End Processing
		if (sensor){
			sensor->Close();
		}
		SafeRelease(sensor);
		SafeRelease(mapper);
		SafeRelease(colorSource);
		SafeRelease(colorReader);
		SafeRelease(depthSource);
		SafeRelease(depthReader);

		thread.join();
	}

	void pcl::KinectGrabber::start()
	{
		// Open Color Frame Reader
		result = colorSource->OpenReader(&colorReader);
		if (FAILED(result)){
			throw std::exception("Exception : IColorFrameSource::OpenReader()");
		}

		// Open Depth Frame Reader
		result = depthSource->OpenReader(&depthReader);
		if (FAILED(result)){
			throw std::exception("Exception : IDepthFrameSource::OpenReader()");
		}

		running = true;

		thread = boost::thread(&KinectGrabber::threadFunction, this);
	}

	void pcl::KinectGrabber::stop()
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		quit = true;
		running = false;

		lock.unlock();
	}

	bool pcl::KinectGrabber::isRunning() const
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		return running;

		lock.unlock();
	}

	std::string pcl::KinectGrabber::getName() const{
		return std::string("KinectGrabber");
	}

	float pcl::KinectGrabber::getFramesPerSecond() const {
		return 30.0f;
	}

	void pcl::KinectGrabber::threadFunction()
	{
		while (!quit)
		{
			boost::unique_lock<boost::mutex> lock(mutex);

			// Acquire Latest Color Frame
			IColorFrame* colorFrame = nullptr;
			result = colorReader->AcquireLatestFrame(&colorFrame);
			if (SUCCEEDED(result)){
				// Retrieved Color Data
				result = colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size(), reinterpret_cast<BYTE*>(colorBuffer.data()), ColorImageFormat::ColorImageFormat_Bgra);
				if (FAILED(result)){
					throw std::exception("Exception : IColorFrame::CopyConvertedFrameDataToArray()");
				}
			}
			SafeRelease(colorFrame);

			// Acquire Latest Depth Frame
			IDepthFrame* depthFrame = nullptr;
			result = depthReader->AcquireLatestFrame(&depthFrame);
			if (SUCCEEDED(result))
			{
				result = depthFrame->get_DepthMinReliableDistance(&depthMinDistance);
				if (FAILED(result)){
					throw std::exception("Exception : IFrameDescription::get_DepthMinReliableDistance()");
				}

				result = depthFrame->get_DepthMaxReliableDistance(&depthMaxDistance);
				if (FAILED(result)){
					throw std::exception("Exception : IFrameDescription::get_DepthMaxReliableDistance()");
				}

				// Retrieved Depth Data
				result = depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);
				if (FAILED(result)){
					throw std::exception("Exception : IDepthFrame::CopyFrameDataToArray()");
				}
			}
			SafeRelease(depthFrame);

			lock.unlock();

			if (signal_PointXYZ->num_slots() > 0) 
				signal_PointXYZ->operator()(convertDepthToPointXYZ(&depthBuffer[0]));

			if (signal_PointXYZRGB->num_slots() > 0) 
				signal_PointXYZRGB->operator()(convertRGBDepthToPointXYZRGB((RGBQUAD*)&colorBuffer[0], &depthBuffer[0]));

			if (signal_FrameBuffer->num_slots() > 0)
				signal_FrameBuffer->operator()(handleFrameBuffer(colorBuffer, depthBuffer));
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::KinectGrabber::convertDepthToPointXYZ(UINT16* depthBuffer)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

		cloud->width = static_cast<uint32_t>(depthWidth);
		cloud->height = static_cast<uint32_t>(depthHeight);
		cloud->is_dense = false;

		cloud->points.resize(cloud->height * cloud->width);

		pcl::PointXYZ* pt = &cloud->points[0];
		for (int y = 0; y < depthHeight; y++){
			for (int x = 0; x < depthWidth; x++, pt++){
				pcl::PointXYZ point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
				point.x = cameraSpacePoint.X;
				point.y = cameraSpacePoint.Y;
				point.z = cameraSpacePoint.Z;

				*pt = point;
			}
		}

		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl::KinectGrabber::convertRGBDepthToPointXYZRGB(RGBQUAD* colorBuffer, UINT16* depthBuffer)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

		cloud->width = static_cast<uint32_t>(depthWidth);
		cloud->height = static_cast<uint32_t>(depthHeight);
		cloud->is_dense = false;

		cloud->points.resize(cloud->height * cloud->width);

		pcl::PointXYZRGB* pt = &cloud->points[0];
		for (int y = 0; y < depthHeight; y++)
		{
			for (int x = 0; x < depthWidth; x++, pt++)
			{
				pcl::PointXYZRGB point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
				ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
				mapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint);
				int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
				int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));
				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)){
					RGBQUAD color = colorBuffer[colorY * colorWidth + colorX];
					point.b = color.rgbBlue;
					point.g = color.rgbGreen;
					point.r = color.rgbRed;
				}

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)){
					point.x = cameraSpacePoint.X;
					point.y = cameraSpacePoint.Y;
					point.z = -cameraSpacePoint.Z;
				}

				//std::cout << std::fixed 
				//	<< "v " << x << ' ' << y << ' ' << depth << std::endl 
				//	<< cameraSpacePoint.X << ' ' << cameraSpacePoint.Y << ' ' << cameraSpacePoint.Z << std::endl;

				*pt = point;
			}
		}
		//std::cout << "------ " << std::endl << std::endl;
		return cloud;
	}


	boost::shared_ptr<KinectFrameBuffer> pcl::KinectGrabber::handleFrameBuffer(const std::vector<unsigned char>& color_buffer, const std::vector<UINT16>& depth_buffer)
	{
		std::vector<uint16_t> info;
		info.push_back(colorWidth);
		info.push_back(colorHeight);
		info.push_back(4);			// channels
		info.push_back(depthWidth);
		info.push_back(depthHeight);
		info.push_back(depthMinDistance);
		info.push_back(depthMaxDistance);
		boost::shared_ptr<KinectFrameBuffer> framebuffer(new KinectFrameBuffer(info, color_buffer, depth_buffer));
		return framebuffer;
	}
}