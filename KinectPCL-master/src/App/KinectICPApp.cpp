#include <SDKDDKVer.h>

// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "KinectGrabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>   // TicToc
#include "PcdGrabberFromFile.h"


#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <iostream>

#pragma warning( disable : 4996  )





template <typename PointType>
class Viewer
{
	typedef pcl::PointCloud<PointType> PointCloud;
	typedef typename PointCloud::ConstPtr ConstPtr;

public:
	Viewer(const std::string& files_folder)
		: viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"))
		, viewer_filter(new pcl::visualization::PCLVisualizer("ICP Viewer"))
		, viewer_result(new pcl::visualization::PCLVisualizer("ICP Result"))
		, cloud_prev(new pcl::PointCloud<pcl::PointXYZRGB>)
		, cloud_result(new pcl::PointCloud<pcl::PointXYZRGB>)
	{
		viewer->setSize(600, 320);
		viewer->setPosition(0, 0);
		
		viewer_result->setSize(600, 320);
		viewer_result->setPosition(800, 0);

		viewer_filter->setSize(600, 320);
		viewer_filter->setPosition(800, 335);

		current_index = -1;

		setFolder(files_folder, ".pcd");

		if (file_names.size() > 0)
		{
			PCL_INFO("%d files loaded.\n", file_names.size());
			current_index = 0;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PCDReader reader;
			if (reader.read(file_names[current_index], *cloud_src) < 0)
				PCL_ERROR("Error loading cloud %s.\n", file_names[current_index].c_str());
			else
				cloud_callback(cloud_src);
		}
	}

	void run()
	{
		viewer->registerMouseCallback(&Viewer::mouse_callback, *this);
		viewer->registerKeyboardCallback(&Viewer::keyboard_callback, *this);
		viewer_result->registerKeyboardCallback(&Viewer::keyboard_callback, *this);
		

		while (!viewer->wasStopped())
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));

			viewer->spinOnce();
			viewer_filter->spinOnce();
			viewer_result->spinOnce();

			ConstPtr cloud;

			if (mutex.try_lock())
			{
				buffer.swap(cloud);
				mutex.unlock();
			}

			if (cloud)
			{
				if (!viewer->updatePointCloud(cloud, "Cloud"))
				{
					viewer->addPointCloud(cloud, "Cloud");
					viewer->resetCameraViewpoint("Cloud");
				}

				pcl::console::TicToc time;
				const int max_iterations = 10;


				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);  // point cloud for voxel grid
				const float leaf_size = 0.01f;
				time.tic();
				pcl::VoxelGrid<pcl::PointXYZRGB> sor;
				sor.setInputCloud(cloud);
				sor.setLeafSize(leaf_size, leaf_size, leaf_size);
				sor.filter(*cloud_filtered);
				std::cout << "Filtered cloud : leaf_size( " << leaf_size << ") Cloud size: (" << cloud_filtered->size() << " points) in " << time.toc() << " ms" << std::endl;


				// The Iterative Closest Point algorithm
				time.tic();
				pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
				icp.setMaximumIterations(max_iterations);
				icp.setInputSource(cloud_prev);
				icp.setInputTarget(cloud_filtered);

				// Set the euclidean distance difference epsilon (criterion 3)
				//icp.setEuclideanFitnessEpsilon(1e-5);
				// Set the transformation epsilon (criterion 2)
				//icp.setTransformationEpsilon(1e-2);
				// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
				//icp.setMaxCorrespondenceDistance(0.5);

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZRGB>);  // point cloud for voxel grid
				icp.align(*cloud_icp);

				std::cout << "Applied " << max_iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

				if (icp.hasConverged())
				{
					std::cout << "ICP has converged, score is " << icp.getFitnessScore() << std::endl;
					std::cout << "ICP transformation " << max_iterations << " : cloud_icp -> cloud_in" << std::endl;

#if 1
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);  // point cloud transformed
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_result_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);  // point cloud transformed
					// Defining a rotation matrix and translation vector
					Eigen::Matrix4d transformation_matrix = icp.getFinalTransformation().cast<double>();
					pcl::transformPointCloud(*cloud_result, *cloud_result_transformed, transformation_matrix);
					pcl::transformPointCloud(*cloud, *cloud_transformed, transformation_matrix);

					*cloud_result = *cloud_transformed + *cloud_result_transformed;
					//pcl::copyPointCloud(*cloud_prev, *cloud_result);
#endif
				}
				else
				{
					PCL_ERROR("\n ***** ICP has not converged.\n");
				}

				std::cout << "-------------------------------" << std::endl << std::endl;


				std::stringstream point_count_filtered;
				point_count_filtered << cloud_filtered->size();
				viewer_filter->setWindowName(point_count_filtered.str());

				if (!viewer_filter->updatePointCloud(cloud_filtered, "Cloud_ICP"))
				{
					viewer_filter->addPointCloud(cloud_filtered, "Cloud_ICP");
					viewer_filter->resetCameraViewpoint("Cloud_ICP");
				}

				std::stringstream point_count;
				point_count << cloud_result->size();
				viewer_result->setWindowName(point_count.str());
				if (!viewer_result->updatePointCloud(cloud_result, "Cloud_Result"))
				{
					viewer_result->addPointCloud(cloud_result, "Cloud_Result");
					viewer_result->resetCameraViewpoint("Cloud_Result");
				}
				//pcl::copyPointCloud(*cloud, *cloud_prev);
				pcl::copyPointCloud(*cloud_filtered, *cloud_prev);
			}

			if (GetKeyState(VK_ESCAPE) < 0)
			{
				break;
			}
		}

	}


	void setFolder(const std::string& files_folder, const std::string& ext = ".pcd")
	{
		file_names.clear();
		boost::filesystem::path path_folder(files_folder);

		if (!boost::filesystem::exists(path_folder))
		{
			pcl::console::print_error(
				"[setFolder] There is no files folder at %s",
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

		if (file_names.size() > 0)
			current_index = 0;
		else
			current_index = -1;
	}

private:
	void cloud_callback(const ConstPtr& cloud)
	{
		viewer->setWindowName(file_names[current_index]);
		boost::mutex::scoped_lock lock(mutex);
		buffer = cloud;
	}

	void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
	{
		if (event.getKeyCode() && event.keyDown())
		{
			std::cout << "Key : " << event.getKeyCode() << ", " << (int)(event.getKeyCode()) << std::endl;

			if (event.getKeyCode() == 'a')
			{
				if (current_index > 0)
					--current_index;

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PCDReader reader;
				if (reader.read(file_names[current_index], *cloud_src) < 0)
					PCL_ERROR("Error loading cloud %s.\n", file_names[current_index].c_str());
				else
					cloud_callback(cloud_src);
			}
			else if (event.getKeyCode() == 'd')
			{
				if (current_index < file_names.size() - 2 )
					++current_index;

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PCDReader reader;
				if (reader.read(file_names[current_index], *cloud_src) < 0)
					PCL_ERROR("Error loading cloud %s.\n", file_names[current_index].c_str());
				else
					cloud_callback(cloud_src);
			}
			else if (event.getKeyCode() == 's')
			{
				std::cout << "saving output_point_cloud.pcd" << std::endl;
				pcl::PCDWriter w;
				w.writeBinaryCompressed("output_point_cloud.pcd", *cloud_result);
			}

		}
	}

	void mouse_callback(const pcl::visualization::MouseEvent& event, void*)
	{
		if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && event.getButton() == pcl::visualization::MouseEvent::LeftButton){
			std::cout << "Mouse : " << event.getX() << ", " << event.getY() << std::endl;
		}
	}

	std::vector<std::string> file_names;
	int current_index;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_filter;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_result;
	boost::mutex mutex;
	ConstPtr buffer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prev;  // point cloud for ICP
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_result;// point cloud for ICP result
};


int main(int argc, char* argv[])
{

	std::string folder_input;
	boost::program_options::options_description desc("./PcdIcpFromFile input_files_folder");

	desc.add_options()
		("help", "produce help message")
		("folder_with_files,i", boost::program_options::value<std::string>(&folder_input)->required(), "folder with input pcd files")
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

	Viewer<pcl::PointXYZRGB> viewer(folder_input);
	viewer.run();

	return 0;
}

