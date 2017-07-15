#include <SDKDDKVer.h>

// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "KinectGrabber.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>   // TicToc

#include <iostream>

//int _tmain( int argc, _TCHAR* argv[] )
int main(int argc, char* argv[])
{
	double leaf_size = 0.05f;

	// Create Cloud Viewer
	pcl::visualization::CloudViewer viewer("Point Cloud Viewer");

	// Callback Function to be called when Updating Data
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> function =
		[&viewer, &leaf_size](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		if (!viewer.wasStopped())
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);  // Original point cloud
			
			pcl::console::TicToc time;
			time.tic();
			pcl::VoxelGrid<pcl::PointXYZRGB> sor;
			sor.setInputCloud(cloud);
			sor.setLeafSize(leaf_size, leaf_size, leaf_size);
			sor.filter(*cloud_in);
			std::cout << "\nFiltered cloud : leaf_size( " << leaf_size << ") Cloud size: (" << cloud_in->size() << " points) in " << time.toc() << " ms\n" << std::endl;
			
			viewer.showCloud(cloud_in);
		}
	};

	// Create KinectGrabber
	pcl::Grabber* grabber = new pcl::KinectGrabber();

	// Regist Callback Function
	grabber->registerCallback(function);

	// Start Retrieve Data
	grabber->start();

	int signal = 1;

	while (!viewer.wasStopped())
	{
		// Input Key ( Exit ESC key )
		if (GetKeyState(VK_ESCAPE) < 0)
		{
			break;
		}

		leaf_size += 0.00000001f * signal;

		if (leaf_size < 0.0005)
			signal = 1;
		if (leaf_size > 0.5)
			signal = -1;
	}

	// Stop Retrieve Data
	grabber->stop();

	return EXIT_SUCCESS;
}

