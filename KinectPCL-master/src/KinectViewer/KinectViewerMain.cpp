#if 1
#include <QApplication>
#include "MainWindow.h"
#include <iostream>


int main(int argc, char **argv)
{
	QApplication app(argc, argv);

	MainWindow w;
	w.show();

	return app.exec();
}
#else


// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "KinectGrabber.h"
#include <pcl/visualization/pcl_visualizer.h>


template <typename PointType>
class Viewer
{
	//typedef typename pcl::PointCloud<PointType>::ConstPtr ConstPtr;

public:
	Viewer(pcl::Grabber& grabber)
		: viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"))
		, grabber(grabber)
	{
	}

	void run()
	{
		viewer->registerMouseCallback(&Viewer::mouse_callback, *this);
		viewer->registerKeyboardCallback(&Viewer::keyboard_callback, *this);
		boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&) > callback = boost::bind(&Viewer::cloud_callback, this, _1);
		boost::signals2::connection connection = grabber.registerCallback(callback);

		grabber.start();

		while (!viewer->wasStopped()){
			viewer->spinOnce();

			pcl::PointCloud<PointType>::ConstPtr cloud;

			if (mutex.try_lock()){
				buffer.swap(cloud);
				mutex.unlock();
			}

			if (cloud){
				if (!viewer->updatePointCloud(cloud, "Cloud")){
					viewer->addPointCloud(cloud, "Cloud");
					viewer->resetCameraViewpoint("Cloud");
				}
			}

			if (GetKeyState(VK_ESCAPE) < 0){
				break;
			}
		}

		grabber.stop();

		connection.disconnect();
	}

private:
	void cloud_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
	{
		boost::mutex::scoped_lock lock(mutex);
		buffer = cloud;
	}

	void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
	{
		if (event.getKeyCode() && event.keyDown()){
			std::cout << "Key : " << event.getKeyCode() << std::endl;
		}
	}

	void mouse_callback(const pcl::visualization::MouseEvent& event, void*)
	{
		if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && event.getButton() == pcl::visualization::MouseEvent::LeftButton){
			std::cout << "Mouse : " << event.getX() << ", " << event.getY() << std::endl;
		}
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::Grabber& grabber;
	boost::mutex mutex;
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr buffer;
};

int main(int argc, char* argv[])
{
	pcl::Grabber* grabber = new pcl::KinectGrabber();
	Viewer<pcl::PointXYZRGB> viewer(*grabber);
	viewer.run();

	return 0;
}

#endif