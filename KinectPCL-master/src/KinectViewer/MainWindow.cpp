#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QKeyEvent>
#include <QFileDialog>
#include <QDebug>
#include <QMessageBox>
#include <QStandardItemModel>
#include <iostream>
#include <QTimer>





MainWindow::MainWindow(QWidget *parent) : 
			QMainWindow(parent),
			ui(new Ui::MainWindow),
			currentFileName(QString())
{
	ui->setupUi(this);

	kinectGrabber = new pcl::KinectGrabber();				// Initialize kinect grabber
	kinectGrabber->start();									// Start Retrieve Data

	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&) > callback = boost::bind(&MainWindow::cloud_callback, this, _1);
	boost::signals2::connection connection = kinectGrabber->registerCallback(callback);

	// Set up the QVTK window
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
	ui->qvtkWidget->update();

	connect(this, SIGNAL(cloudUpdate()), this, SLOT(update()));
}



MainWindow::~MainWindow()
{
	kinectGrabber->stop();			// Stop Retrieve Data

	delete ui;
}

void MainWindow::cloud_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
	boost::mutex::scoped_lock lock(mutex);
	buffer = cloud;

	Q_EMIT cloudUpdate();
}

void MainWindow::update()
{
	if (!viewer)
		return;

	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;

	if (mutex.try_lock())
	{
		buffer.swap(cloud);
		mutex.unlock();
	}

	if (cloud.get())
	{
		if (!viewer->updatePointCloud(cloud, "Cloud"))
		{
			viewer->addPointCloud(cloud, "Cloud");
			//viewer->resetCameraViewpoint("Cloud");
			viewer->resetCamera();
		}
		ui->qvtkWidget->update();
	}

	QMainWindow::update();
}


void MainWindow::fileNew()
{
	currentFileName.clear();
}




void MainWindow::fileOpen()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Kinect Stream (*.knt)"));

	if (!fileName.isEmpty())
	{
		currentFileName = fileName;
		Q_EMIT fileOpen(currentFileName);
	}
}




void MainWindow::fileSave()
{
	if (!currentFileName.isEmpty())
	{
		Q_EMIT fileSave(currentFileName);
	}
	else
	{
		fileSaveAs();
	}
}




void MainWindow::fileSaveAs()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "", tr("Kinect Stream (*.knt)"));
	if (!fileName.isEmpty())
	{
		currentFileName = fileName;
		fileSave();
	}
}




void MainWindow::playerPlay()
{
	Q_EMIT play();
}


void MainWindow::playerStop()
{
	Q_EMIT stop();
}


void MainWindow::playerRecord(bool triggered)
{
	Q_EMIT recordToggled(triggered);
}



void MainWindow::playerCapture(bool triggered)
{
	Q_EMIT captureToggled(triggered);
}

void MainWindow::playerTakeShot()
{
	Q_EMIT takeShot();
}


void MainWindow::aboutDialogShow()
{
	QString message
		("<p>Alpha Matting algorithm using Qt and Opengl" \
		"<p><p>" \
		"<br>   [1] K. He, C. Rhemann, C. Rother, X. Tang, J. Sun, A Global Sampling Method for Alpha Matting, CVPR, 2011. <br>" \
		"<br>   [2] C. Tomasi and R. Manduchi, Bilateral Filtering for Gray and Color Images, Proc.IEEE Intel Computer Vision Conference, 1998. <br>" \
		"<br>   [3] K.He, J.Sun, and X.Tang, Guided Image Filtering,  Proc. European Conf.Computer Vision, pp. 1 - 14, 2010. <br>" \
		"<p><p><p>" \
		"<p>Developed by: Diego Mazala, June-2015" \
		"<p>");

	QMessageBox::about(this, tr("Alpha Matting"), message);
}

