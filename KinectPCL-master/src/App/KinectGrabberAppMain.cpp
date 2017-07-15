#include <SDKDDKVer.h>

// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "KinectGrabber.h"
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

//int _tmain( int argc, _TCHAR* argv[] )
int main(int argc, char* argv[])
{
	// Create Cloud Viewer
	pcl::visualization::CloudViewer viewer("Point Cloud Viewer");

	// Callback Function to be called when Updating Data
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> function =
		[&viewer](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		if (!viewer.wasStopped()){
			viewer.showCloud(cloud);
		}
	};

	// Create KinectGrabber
	pcl::Grabber* grabber = new pcl::KinectGrabber();

	// Regist Callback Function
	grabber->registerCallback(function);

	// Start Retrieve Data
	grabber->start();

	while (!viewer.wasStopped())
	{
		// Input Key ( Exit ESC key )
		if (GetKeyState(VK_ESCAPE) < 0){
			break;
		}
	}

	// Stop Retrieve Data
	grabber->stop();

	return EXIT_SUCCESS;
}