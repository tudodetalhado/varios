#include "PcdGrabberFromFile.h"
#include <pcl/io/pcd_io.h>

namespace pcl
{

	pcl::PcdGrabberFromFile::PcdGrabberFromFile()
		: 
		  running(false)
		, quit(false)
		, signal_PointXYZ(nullptr)
		, signal_PointXYZRGB(nullptr)
	{
		setFramesPerSecond(30);
		signal_PointXYZ = createSignal<signal_Kinect2_PointXYZ>();
		signal_PointXYZRGB = createSignal<signal_Kinect2_PointXYZRGB>();
	}

	pcl::PcdGrabberFromFile::~PcdGrabberFromFile() throw()
	{
		stop();

		disconnect_all_slots<signal_Kinect2_PointXYZ>();
		disconnect_all_slots<signal_Kinect2_PointXYZRGB>();


		thread.join();
	}

	void pcl::PcdGrabberFromFile::setFolder(const std::string& files_folder, const std::string& ext)
	{
		file_names.clear();
		boost::filesystem::path path_folder(files_folder);

		if (!boost::filesystem::exists(path_folder))
		{
			pcl::console::print_error(
				"[PcdGrabber] There is no files folder at %s",
				path_folder.c_str());
			return;
		}

		if (boost::filesystem::is_regular_file(path_folder)
			&& (path_folder.extension().string() == ext))
		{
			file_names.push_back(path_folder.string());
		}
		else
		{
			boost::filesystem::directory_iterator diter(path_folder), dend;
			for (; diter != dend; diter++)
			{
				if ((diter->path().extension() == ext)
					&& (boost::filesystem::is_regular_file(diter->path())))
					file_names.push_back(diter->path().string());
			}
			std::sort(file_names.begin(), file_names.end());
		}
	}

	void pcl::PcdGrabberFromFile::setFramesPerSecond(float frame_rate_fps)
	{
		frame_rate = frame_rate_fps;
		sleep_ms = 1.0f / frame_rate * 1000;
	}

	void pcl::PcdGrabberFromFile::start()
	{
		running = true;
		frame_idx = 0;
		thread = boost::thread(&PcdGrabberFromFile::threadFunction, this);
	}

	void pcl::PcdGrabberFromFile::stop()
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		quit = true;
		running = false;

		lock.unlock();
	}

	bool pcl::PcdGrabberFromFile::isRunning() const
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		return running;

		lock.unlock();
	}

	std::string pcl::PcdGrabberFromFile::getName() const
	{
		return std::string("PcdGrabberFromFile");
	}

	float pcl::PcdGrabberFromFile::getFramesPerSecond() const 
	{
		return frame_rate;
	}

	void pcl::PcdGrabberFromFile::threadFunction()
	{
		while ((frame_idx < file_names.size()) && !quit)
		{
			std::string current_filename;
			boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_ms));

			boost::unique_lock<boost::mutex> lock(mutex);
			{
				frame_idx++;
				if (frame_idx < file_names.size())
					current_filename = file_names[frame_idx];
			}
			lock.unlock();


			if (signal_PointXYZ->num_slots() > 0)
			{
				if (!current_filename.empty())
				{
					signal_PointXYZ->operator()(loadPcdFileXYZ(current_filename));
				}
			}

			if (signal_PointXYZRGB->num_slots() > 0)
			{
				if (!current_filename.empty())
				{
					signal_PointXYZRGB->operator()(loadPcdFileXYZRGB(current_filename));
				}
					
			}
		}
	}


	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::PcdGrabberFromFile::loadPcdFileXYZ(const std::string& filename)
	{
		pcl::PCLPointCloud2::Ptr cloud_pcd(new pcl::PCLPointCloud2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

		pcl::PCDReader pcd;
		if (pcd.read(filename, *cloud_pcd) < 0)
		{
			pcl::console::print_error("[PcdGrabberFromFile::loadPcdFileXYZ] Failed to load file %s", filename.c_str());
		}

		pcl::fromPCLPointCloud2(*cloud_pcd, *cloud);

		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl::PcdGrabberFromFile::loadPcdFileXYZRGB(const std::string& filename)
	{
		pcl::PCLPointCloud2::Ptr cloud_pcd(new pcl::PCLPointCloud2);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		
		pcl::PCDReader pcd;
		if (pcd.read(filename, *cloud_pcd) < 0)
		{
			pcl::console::print_error("[PcdGrabberFromFile::loadPcdFileXYZRGB] Failed to load file %s", filename.c_str());
		}

		pcl::fromPCLPointCloud2(*cloud_pcd, *cloud);

		return cloud;
	}



}