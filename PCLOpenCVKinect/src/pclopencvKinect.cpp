#include "pclopencvKinect.h"
#include "../build/ui_pclopencvKinect.h"
#include "cloudData.h"

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
  ui->qvtkWidget->update ();

  // First call
  firstCall = true;

  // Show stream
  StopStream = false;
  
  // Scale to fit
  ui->lblRGBimage->setScaledContents(true);
  // Load face detector classifier
  myDetector.loadClassifier();
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
		// Track 3D face based on 2D image
		else {
            KinectRGB = cv::Mat(cloudHeight,cloudWidth,CV_8UC3); // Allocate memory
			// Convert KinectRGB image to Mat
			cv::Vec3b color;
			unsigned char *tmp = (unsigned char*) (KinectRGB.data);
			if(!cloudKinect->empty()) {
				for(int h=0;h<KinectRGB.rows;h++) {
					for(int w=0;w<KinectRGB.cols;w++) {
						pcl::PointXYZRGBA C = cloudKinect->at(w,h);						
						color[0] = C.b; // blue
						color[1] = C.g; // green
						color[2] = C.r; // red
						KinectRGB.at<cv::Vec3b>(cv::Point(w,h)) = color;
					}
				}
			}

			// Passthrough filter
			pcl::PassThrough<pcl::PointXYZRGBA> passFilter;
			// Face tracking
			if(faceTracking == true) {								
				cv::cvtColor(KinectRGB,KinectGray,CV_BGR2GRAY); // Convert to grayscale
				er = myDetector.detection(KinectGray); // Detection				
				if(er == 0) { // Check if face detected					
					cv::rectangle(KinectRGB,myDetector.detectedFace[0],cv::Scalar(0,0,255),1,8,0); // Draw detected face
					// New pointcloud for detected face
					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFace (new pcl::PointCloud<pcl::PointXYZRGBA>);
					cloudFace->width = myDetector.detectedFace[0].width;
					cloudFace->height = myDetector.detectedFace[0].height;
					cloudFace->points.resize(cloudFace->width*cloudFace->height);
					cloudFace->is_dense = false;
					bool t = cloudFace->isOrganized();
					// Fill the point cloud of the extracted face
					for(int h=0;h<myDetector.detectedFace[0].height;h++) {
						for(int w=0;w<myDetector.detectedFace[0].width;w++) {
							cloudFace->at(w,h) = 
							cloudKinect->at(w+myDetector.detectedFace[0].x,h+myDetector.detectedFace[0].y);							
						}
					}
					// Filter
					passFilter.setInputCloud(cloudFace);
					passFilter.setFilterFieldName("z");
					passFilter.setFilterLimits(0.0,(float)(ui->sldClipDistance->value() / 1000.0));
					ui->lblClipDistance->setText(QString("Clip far data [mm]: \n") +
											     QString::number((float)(ui->sldClipDistance->value() / 1000.0),'f',3));
					passFilter.filter(*cloudFace);
					// Update face point cloud
					viewer->updatePointCloud (cloudFace,"cloud");
					ui->qvtkWidget->update ();
				}				
			}
			else {			
				// Filter
				passFilter.setInputCloud(cloudKinect);
				passFilter.setFilterFieldName("z");
				passFilter.setFilterLimits(0.0,(float)(ui->sldClipDistance->value() / 1000.0));
				ui->lblClipDistance->setText(QString("Clip far data [mm]: \n") +
											 QString::number((float)(ui->sldClipDistance->value() / 1000.0),'f',3));
				passFilter.filter(*cloudKinect);
				// Update entire point cloud
				viewer->updatePointCloud (cloudKinect,"cloud");
				ui->qvtkWidget->update ();
			}

            // OpenCV to QImage (frameImage)
            cv::cvtColor(KinectRGB,KinectRGB,CV_BGR2RGB);
            QImage qimgKinectRGB((uchar*)KinectRGB.data,KinectRGB.cols,KinectRGB.rows,KinectRGB.step,QImage::Format_RGB888);
            // Update label
			ui->lblRGBimage->setPixmap(QPixmap::fromImage(qimgKinectRGB));		    
		}		
	}
}

KinectViewer::~KinectViewer ()
{
  // Stop Kinect grabber
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

void KinectViewer::on_btnTrackFace_toggled(bool checked)
{
    if(checked == true)
        faceTracking = true;
    else
        faceTracking = false;
}
