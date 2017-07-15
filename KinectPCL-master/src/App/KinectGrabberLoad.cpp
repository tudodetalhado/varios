// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "PcdGrabberFromFile.h"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>



int main(int argc, char* argv[])
{
	std::string folder_input;
	std::string file_format = "pcd";
	boost::program_options::options_description desc("./KinectGrabberLoad input_files_folder");

	desc.add_options()
		("help", "produce help message")
		("folder_with_files,i", boost::program_options::value<std::string>(&folder_input)->required(), "folder with input pcd files")
		("file_format,f", boost::program_options::value<std::string>(&file_format), "file format (pcd or knt)")
		;

	boost::program_options::positional_options_description p;
	p.add("folder_with_pcd_files", 1);

	boost::program_options::variables_map vm;
	try
	{
		boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
		boost::program_options::notify(vm);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		std::cout << desc << std::endl;
		return 1;
	}
	if (vm.count("help"))
	{
		std::cout << desc << std::endl;
		return 0;
	}


	// Create KinectGrabber
	pcl::Grabber* grabber = nullptr;

	//if (file_format.find("knt"))		// knt format
	//{
	//	grabber = new pcl::KinectGrabberFromFile();
	//	((pcl::KinectGrabberFromFile*)grabber)->setFolder(folder_input);
	//}
	//else								// pcd format
	{
		grabber = new pcl::PcdGrabberFromFile();
		((pcl::PcdGrabberFromFile*)grabber)->setFolder(folder_input);
	}



	// Create Cloud Viewer
	pcl::visualization::CloudViewer viewer("PCD Grabber From File");

	// Callback Function to be called when Updating Data
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> show_cb =
		[&viewer](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		if (!viewer.wasStopped()){
			viewer.showCloud(cloud);
		}
	};
	grabber->registerCallback(show_cb);

	
	// Start Retrieve Data
	grabber->start();

	while (!viewer.wasStopped())
	{
		// Input Key ( Exit ESC key )
		if (GetKeyState(VK_ESCAPE) < 0)
		{
			break;
		}
	}

	// Stop Retrieve Data
	grabber->stop();

	return EXIT_SUCCESS;
}



