#include "pclKinect.h"
#include "../build/ui_pclKinect.h"
#include "cloudData.h"

std::vector<float> cloudX;
std::vector<float> cloudY;
std::vector<float> cloudZ;
std::vector<unsigned long>  cloudRGB;
int cloudWidth;
int cloudHeight;

KinectViewer::KinectViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::KinectViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("Kinect viewer");



  // Timer for 3D/UI update
  tmrTimer = new QTimer(this);
  connect(tmrTimer,SIGNAL(timeout()),this,SLOT(processFrameAndUpdateGUI()));
  tmrTimer->start(20); // msec

  // Setup Kinect
  bCopying = bRun = false;
  grabber = NULL;

  // Run Kinect grabber
  er = run(); 

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update();

  // First call
  firstCall = true;

  // Show stream
  StopStream = false;
}

void KinectViewer::processFrameAndUpdateGUI() {
    if(bCopying == false && StopStream == false) {
		// Setup cloud
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudKinect (new pcl::PointCloud<pcl::PointXYZRGBA>);
		cloudKinect->width = cloudWidth;
		cloudKinect->height = cloudHeight;
		cloudKinect->points.resize(cloudWidth*cloudHeight);
		cloudKinect->is_dense = false;
		// Fill cloud
        float *pX = &cloudX[0];
        float *pY = &cloudY[0];
        float *pZ = &cloudZ[0];
		unsigned long *pRGB = &cloudRGB[0];
		for(int i=0;i<cloudKinect->points.size();i++,pX++,pY++,pZ++,pRGB++) {
			cloudKinect->points[i].x = (*pX);
			cloudKinect->points[i].y = (*pY);
			cloudKinect->points[i].z = (*pZ);
			cloudKinect->points[i].rgba = (*pRGB);
		}
		// Add point cloud on first call
		if(firstCall == true) {
            viewer->addPointCloud(cloudKinect,"cloud");
            viewer->resetCamera();
            ui->qvtkWidget->update();
			firstCall = false;
		}
		// Update point cloud
		else {
			// Passthrough filter
			pcl::PassThrough<pcl::PointXYZRGBA> passFilter;
			passFilter.setInputCloud(cloudKinect);
			passFilter.setFilterFieldName("z");
            passFilter.setFilterLimits(0.0,(float)(ui->sldClipDistance->value() / 1000.0));
            ui->lblClipDistance->setText(QString("Clip far data [mm]: \n") +
                                         QString::number((float)(ui->sldClipDistance->value() / 1000.0),'f',3));
			passFilter.filter(*cloudKinect);
			viewer->updatePointCloud (cloudKinect,"cloud");
			ui->qvtkWidget->update ();
		}
	}
}

KinectViewer::~KinectViewer ()
{
  er = stop();
  delete ui;
}

void KinectViewer::on_btnStopStream_toggled(bool checked)
{
    if(checked == true){
        StopStream = true;
        ui->btnStopStream->setText("Stream stopped");
    }
    else {
        StopStream = false;
        ui->btnStopStream->setText("Stream running");
    }
}

void KinectViewer::on_btnResetCamera_clicked()
{
    viewer->resetCamera();
    ui->qvtkWidget->update();
}
