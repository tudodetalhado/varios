// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "KinectGrabber.h"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>



template<typename T>
static void vector_write(std::ostream& out_file, const std::vector<T>& data)
{
	const std::size_t count = data.size();
	out_file.write(reinterpret_cast<const char*>(&count), sizeof(std::size_t));
	out_file.write(reinterpret_cast<const char*>(&data[0]), count * sizeof(T));
}

template<typename T>
static void vector_read(std::istream& in_file, std::vector<T>& data)
{
	std::size_t count;
	in_file.read(reinterpret_cast<char*>(&count), sizeof(std::size_t));
	data.resize(count);
	in_file.read(reinterpret_cast<char*>(&data[0]), count * sizeof(T));
}


std::string folder_output;
std::string file_format = "pcd";
uint32_t number_to_grab = 1, count = 0;
bool binary_format = true;


void savecloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
	if (count < number_to_grab)
	{
		std::stringstream filename;
		filename << folder_output << '/' << boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::local_time()) << ".pcd";
		std::cout << count << " - " << filename.str() << std::endl;
		pcl::PCDWriter w;
		w.writeBinaryCompressed(filename.str(), *cloud);
		++count;
	}
}




void save_kinect_frame_buffer(const boost::shared_ptr<const pcl::KinectFrameBuffer>& frame_buffer)
{
	if (count < number_to_grab)
	{
		std::stringstream filename;
		filename << folder_output << '/' << count << ".knt";
		std::cout << count << " - " << filename.str() << std::endl;
		
		std::ofstream out;
		out.open(filename.str(), std::ofstream::out | std::ofstream::binary);
		{
			vector_write(out, frame_buffer->info);
			vector_write(out, frame_buffer->color);
			vector_write(out, frame_buffer->depth);
		}
		out.close();
		++count;
	}

}




int main(int argc, char* argv[])
{
	boost::program_options::options_description desc("./KinectGrabberSave folder_output");

	desc.add_options()
		("help", "produce help message")
		("folder_output,o", boost::program_options::value<std::string>(&folder_output)->required(), "output pcd folder")
		("file_format,f", boost::program_options::value<std::string>(&file_format), "file format (pcd or knt)")
		("number,n", boost::program_options::value<uint32_t>(&number_to_grab)->default_value(1), "number of pcd to grab")
		("binary_format,b", boost::program_options::value<bool>(&binary_format)->default_value(true), "binary format")
		;

	boost::program_options::positional_options_description p;
	p.add("folder_output", 1);

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


	if (!boost::filesystem::is_directory(folder_output))		// Check if the directory exists
	{
		std::cout << "Creating folder '" << folder_output << "'" << std::endl;
		if (!boost::filesystem::create_directory(folder_output))
		{
			std::cerr << "Could not create output folder '" << folder_output << "'. Abort." << std::endl;
			return EXIT_FAILURE;
		}
	}

	// Create KinectGrabber
	pcl::Grabber* grabber = new pcl::KinectGrabber();


	// Create Cloud Viewer
	pcl::visualization::CloudViewer viewer("Point Cloud Viewer");

	// Callback Function to be called when Updating Data
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> show_cb =
		[&viewer](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		if (!viewer.wasStopped()){
			viewer.showCloud(cloud);
		}
	};
	grabber->registerCallback(show_cb);


	if (file_format.find("knt") != std::string::npos)		// knt format
	{
		// Callback Function to be called when Updating Data
		boost::function<void(const boost::shared_ptr<const pcl::KinectFrameBuffer>&)> save_kinect_frame_buffer_cb =
			[&viewer](const boost::shared_ptr<const pcl::KinectFrameBuffer> frame_buffer)
		{
			save_kinect_frame_buffer(frame_buffer);
		};
		grabber->registerCallback(save_kinect_frame_buffer_cb);
	}
	else								// pcd format
	{
		// Callback Function to be called when Updating Data
		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> save_cloud_cb =
			[&viewer](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
		{
			savecloud(cloud);
		};
		grabber->registerCallback(save_cloud_cb);
	}


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
