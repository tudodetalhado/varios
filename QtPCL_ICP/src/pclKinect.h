#pragma once

#ifndef KINECTVIEWER_H
#define KINECTVIEWER_H

#include <iostream>
#include "cloudData.h"

// Qt
#include <QtWidgets/QMainWindow>
#include <QTimer>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include "kinect2_grabber.h"

namespace Ui
{
  class KinectViewer;
}

class KinectViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit KinectViewer (QWidget *parent = 0);
  ~KinectViewer ();

public:
  // Grabber
  //pcl::Grabber* grabber;

  // Kinect2Grabber
  boost::shared_ptr<pcl::Grabber> grabber;
 
  // Running
  bool bRun; 
  // Copying
  bool bCopying;   
  // Error
  int er;
  // First call
  bool firstCall;
  // Add cloud
  bool addCloud;
  // Remove all clouds
  bool removeClouds;
  // Register clouds
  bool registerClouds;
  // Transformation matrix
  Eigen::Matrix4f finalTransformation;
  // Angles
  float angleX,angleY,angleZ;

public:
	// Point cloud callback
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		if (bRun) {
			while(bCopying) 
				Sleep(0);
			bCopying=true;
			// Size of cloud
			cloudWidth = cloud->width;
			cloudHeight = cloud->height;
            // Resize the XYZ and RGB point vector
            cloudX.resize(cloud->height*cloud->width);
            cloudY.resize(cloud->height*cloud->width);
            cloudZ.resize(cloud->height*cloud->width);
            cloudRGB.resize(cloud->height*cloud->width);
            // Assign pointers to copy data
            float *pX = &cloudX[0];
            float *pY = &cloudY[0];
            float *pZ = &cloudZ[0];
            unsigned long *pRGB = &cloudRGB[0];
			
            // Copy data (using pcl::copyPointCloud, the color stream jitters!!! Why?)
            for (int j = 0;j<cloud->height;j++){
                for (int i = 0;i<cloud->width;i++,pX++,pY++,pZ++,pRGB++) {
                    pcl::PointXYZRGBA P = cloud->at(i,j);
                    (*pX) = P.x;
                    (*pY) = P.y;
                    (*pZ) = P.z;
                    (*pRGB) = P.rgba;
                }
            }
			// Data copied			
			bCopying = false;			
		}
	}
	
	// Run Kinect
	int run() {
		if(bRun == true)
			return -1;
		
        //grabber = new pcl::OpenNIGrabber("",pcl::OpenNIGrabber::Mode::OpenNI_VGA_30Hz, pcl::OpenNIGrabber::Mode::OpenNI_VGA_30Hz);
        grabber = boost::make_shared<pcl::Kinect2Grabber>();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&KinectViewer::cloud_cb_, this, _1);		
		
		grabber->registerCallback(f);
		
		// Start grabber
		grabber->start(); 
		// Running
		bRun = true;
		return 0;
	}

	// Stop Kinect
	int stop() {
		if(bRun == false)
			return -1;
		// Not running
		bRun = false; 
		// Stop grabber
		grabber->stop(); 
		return 0;
	}	

private:
  QTimer *tmrTimer;

public:
  bool StopStream;

public slots:
  void processFrameAndUpdateGUI();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; 
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerRegistration; 

private slots:
  void on_btnStopStream_toggled(bool checked);

  void on_btnResetCamera_clicked();

  void on_btnAddCloud_clicked();

  void on_btnRemoveCloud_clicked();

  void on_btnRegistration_clicked();

private:
  Ui::KinectViewer *ui;  

};

#endif // KINECTVIEWER_H
