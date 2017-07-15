#ifndef _MAIN_WINDOW_H_
#define _MAIN_WINDOW_H_


#include <QMainWindow>



// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// Kinect grabber
#include "KinectGrabber.h"



class QKinectPlayerCtrl;

namespace Ui 
{
	class MainWindow;
}


class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	
	explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


public Q_SLOTS:
	void fileNew();
	void fileOpen();
	void fileSave();
	void fileSaveAs();
	void playerPlay();
	void playerStop();
	void playerRecord(bool triggered);
	void playerCapture(bool triggered);
	void playerTakeShot();

	void aboutDialogShow();

	void update();

Q_SIGNALS:
	void fileOpen(QString);
	void fileSave(QString);
	void recordToggled(bool);
	void captureToggled(bool);
	void takeShot();
	void play();
	void stop();
	void cloudUpdate();

private:

	void cloud_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::Grabber* kinectGrabber;
	boost::mutex mutex;
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr buffer;

	Ui::MainWindow *ui;
	QString currentFileName;
};

#endif // _MAIN_WINDOW_H_