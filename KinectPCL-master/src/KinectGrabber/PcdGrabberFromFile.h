
#ifndef __PCD_GRABBER_FROM_FILE_H__
#define __PCD_GRABBER_FROM_FILE_H__

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

namespace pcl
{
	

	struct pcl::PointXYZ;
	struct pcl::PointXYZRGB;
	template <typename T> class pcl::PointCloud;

	class PcdGrabberFromFile : public pcl::Grabber
	{
		public:
			PcdGrabberFromFile();
			virtual ~PcdGrabberFromFile() throw ();
			virtual void setFolder(const std::string& files_folder, const std::string& ext = ".pcd");
			virtual void start();
			virtual void stop();
			virtual bool isRunning() const;
			virtual std::string getName() const;
			virtual float getFramesPerSecond() const;
			virtual void setFramesPerSecond(float frame_rate_fps);

			typedef void ( signal_Kinect2_PointXYZ )( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>& );
			typedef void ( signal_Kinect2_PointXYZRGB )( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& );

		protected:
			boost::signals2::signal<signal_Kinect2_PointXYZ>* signal_PointXYZ;
			boost::signals2::signal<signal_Kinect2_PointXYZRGB>* signal_PointXYZRGB;

			
			pcl::PointCloud<pcl::PointXYZ>::Ptr loadPcdFileXYZ(const std::string& filename);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPcdFileXYZRGB(const std::string& filename);
			


			boost::thread thread;
			mutable boost::mutex mutex;

			void threadFunction();

			bool quit;
			bool running;

			std::vector<std::string> file_names;
			size_t frame_idx;

			float frame_rate;
			uint32_t sleep_ms;
	};

}

#endif // __PCD_GRABBER_FROM_FILE_H__

