// KinectGrabber is pcl::Grabber to retrieve the point cloud data from Kinect v2 using Kinect for Windows SDK 2.x.
// This source code is licensed under the MIT license. Please see the License in License.txt.

// This code has been derived from https://github.com/UnaNancyOwen/KinectGrabber

#ifndef __KINECT_GRABBER_H__
#define __KINECT_GRABBER_H__

#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace pcl
{
	struct pcl::PointXYZ;
	struct pcl::PointXYZRGB;
	template <typename T> class pcl::PointCloud;

	template<class Interface>
	inline void SafeRelease( Interface *& IRelease )
	{
		if( IRelease != NULL )
		{
			IRelease->Release();
			IRelease = NULL;
		}
	}

	struct KinectFrameBuffer
	{
		// info : [0] color_width
		// info : [1] color_height
		// info : [2] color_channels
		// info : [3] depth_width
		// info : [4] depth_width
		// info : [5] depth_min_distance
		// info : [6] depth_max_distance
		enum
		{
			ColorWidth = 0,
			ColorHeight,
			ColorChannels,
			DepthWidth,
			DepthHeight,
			DepthMinDistance,
			DepthMaxDistance,
			eTotal
		};

		KinectFrameBuffer()
		{
			reset();
		}
		KinectFrameBuffer(const std::vector<unsigned short>& _info,
			const std::vector<unsigned char>& _color,
			const std::vector<unsigned short>& _depth) :
			info(_info), color(_color), depth(_depth){}


		void clear()
		{
			info.clear();
			color.clear();
			depth.clear();
		}

		void reset()
		{
			info.resize(eTotal, 0);
			color.resize(1920 * 1080 * 4, 0);
			depth.resize(512 * 424, 0);
		}

		unsigned short color_width() const { return info[ColorWidth]; }
		unsigned short color_height() const { return info[ColorHeight]; }
		unsigned short depth_width() const { return info[DepthWidth]; }
		unsigned short depth_height() const { return info[DepthHeight]; }
		unsigned short depth_min_distance() const { return info[DepthMinDistance]; }
		unsigned short depth_max_distance() const { return info[DepthMaxDistance]; }

		std::vector<unsigned short> info;
		std::vector<unsigned char> color;
		std::vector<unsigned short> depth;
	};

	class KinectGrabber : public pcl::Grabber
	{
		public:
			KinectGrabber();
			virtual ~KinectGrabber() throw ();
			virtual void start();
			virtual void stop();
			virtual bool isRunning() const;
			virtual std::string getName() const;
			virtual float getFramesPerSecond() const;

			typedef void ( signal_Kinect2_PointXYZ )( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>& );
			typedef void ( signal_Kinect2_PointXYZRGB )( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& );

			typedef void (signal_Kinect2_FrameBuffer)(const boost::shared_ptr<const KinectFrameBuffer>&);

		protected:
			boost::signals2::signal<signal_Kinect2_PointXYZ>* signal_PointXYZ;
			boost::signals2::signal<signal_Kinect2_PointXYZRGB>* signal_PointXYZRGB;

			boost::signals2::signal<signal_Kinect2_FrameBuffer>* signal_FrameBuffer;

			pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ( UINT16* depthBuffer );
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB( RGBQUAD* colorBuffer, UINT16* depthBuffer );
									
			boost::shared_ptr<KinectFrameBuffer> handleFrameBuffer(const std::vector<unsigned char>& color_buffer, const std::vector<UINT16>& depth_buffer);

			boost::thread thread;
			mutable boost::mutex mutex;

			void threadFunction();

			bool quit;
			bool running;

			HRESULT result;
			IKinectSensor* sensor;
			ICoordinateMapper* mapper;
			IColorFrameSource* colorSource;
			IColorFrameReader* colorReader;
			IDepthFrameSource* depthSource;
			IDepthFrameReader* depthReader;

			int colorWidth;
			int colorHeight;
			std::vector<unsigned char> colorBuffer;

			int depthWidth;
			int depthHeight;
			uint16_t depthMinDistance;
			uint16_t depthMaxDistance;
			std::vector<UINT16> depthBuffer;
	};

}

#endif // __KINECT_GRABBER_H__

