#include "pclKinect.h"
#include "../build/ui_pclKinect.h"
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

  viewerRegistration.reset (new pcl::visualization::PCLVisualizer ("viewerRegistration", false));
  ui->qvtkWidgetRegistration->SetRenderWindow (viewerRegistration->getRenderWindow ());
  viewerRegistration->setupInteractor (ui->qvtkWidgetRegistration->GetInteractor (), ui->qvtkWidgetRegistration->GetRenderWindow ());
  ui->qvtkWidgetRegistration->update ();

  // First call
  firstCall = true;
  // Add cloud
  addCloud = false;
  // Remove all clouds
  removeClouds = false;  
  // Register clouds
  registerClouds = false;  
  // Add text
  viewerRegistration->addText("ICP", 10, 20, 16, 1.0, 1.0, 1.0, "fitness_score",0);
  // Show stream
  StopStream = false;
  // Init transformation matrix
  finalTransformation = Eigen::Matrix4f::Identity();
}

void KinectViewer::processFrameAndUpdateGUI() {
    if(bCopying == false && StopStream == false) {	
		// Setup clouds for registration
		static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudRegistration1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
		static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudRegistration2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
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
        // Add cloud to the other widget
        if(addCloud == true){
			static unsigned char counter = 0;
			switch(counter) {
				case 0:
					{	
						cloudRegistration1->width = cloudKinect->width;
						cloudRegistration1->height = cloudKinect->height;
						cloudRegistration1->points.resize(cloudRegistration1->width*cloudRegistration1->height);
						cloudRegistration1->is_dense = false;
						// Copy cloud
						pcl::copyPointCloud(*cloudKinect,*cloudRegistration1);
						viewerRegistration->addPointCloud(cloudRegistration1,"cloud1");
						counter++;
					}
					break;
				case 1:
					{
						cloudRegistration2->width = cloudKinect->width;
						cloudRegistration2->height = cloudKinect->height;
						cloudRegistration2->points.resize(cloudRegistration2->width*cloudRegistration2->height);
						cloudRegistration2->is_dense = false;
						// Copy cloud
						pcl::copyPointCloud(*cloudKinect,*cloudRegistration2);
						viewerRegistration->addPointCloud(cloudRegistration2,"cloud2");
						counter = 0;
					}
					break;
				default:
					break;
			} 
			viewerRegistration->resetCamera();
            ui->qvtkWidgetRegistration->update();
            addCloud = false;        
		}
		// Remove clouds
		if(removeClouds == true) {
			viewerRegistration->removeAllPointClouds();
			viewerRegistration->resetCamera();
			ui->qvtkWidgetRegistration->update();
            removeClouds = false;
			// Clear indicators
			ui->txtTransformation->clear();
			ui->txtAngleX->clear();
			ui->txtAngleY->clear();
			ui->txtAngleZ->clear();
			ui->txtTx->clear();
			ui->txtTy->clear();
			ui->txtTz->clear();
		}
        // Register clouds
        if(registerClouds == true){
		
			// Single iteration steps
			static int iterations = 0;
			// Create instance of ICP and set the clouds
			pcl::IterativeClosestPoint<pcl::PointXYZRGBA,pcl::PointXYZRGBA> icp;
			icp.setInputTarget(cloudRegistration1);
			icp.setInputCloud(cloudRegistration2);
			icp.setMaximumIterations(1);			
			
			std::stringstream ss,ss1;						
			// Align
			icp.align(*cloudRegistration2);	
			iterations++;			
			finalTransformation *= icp.getFinalTransformation();
			ss.str ("");
			ss1.str("");
			ss << icp.getFitnessScore();
			ss1 << iterations;
			std::string fitness = "Fitness score = " + ss.str () + ", iteration = " + ss1.str();
			viewerRegistration->updateText(fitness, 10, 20, 16, 1.0, 1.0, 1.0, "fitness_score");
			viewerRegistration->updatePointCloud(cloudRegistration2,"cloud2");
			ui->qvtkWidgetRegistration->update();			
			// Stop under the following conditions
			if(iterations >= ui->boxMaxIterations->value()) {
				ui->txtTransformation->appendPlainText(QString("[  ") + 
								     				   QString::number(finalTransformation(0,0),'f',3) +
													   QString("  ") +
													   QString::number(finalTransformation(0,1),'f',3) +
													   QString("  ") +
													   QString::number(finalTransformation(0,2),'f',3) +
													   QString("  ") +
													   QString::number(finalTransformation(0,3),'f',3) +
													   QString("\n  ") +
													   QString::number(finalTransformation(1,0),'f',3) +
													   QString("  ") +
													   QString::number(finalTransformation(1,1),'f',3) +
													   QString("  ") +
													   QString::number(finalTransformation(1,2),'f',3) +
													   QString("  ") +
													   QString::number(finalTransformation(1,3),'f',3) +
													   QString("\n  ") +
													   QString::number(finalTransformation(2,0),'f',3) +
													   QString("  ") +
													   QString::number(finalTransformation(2,1),'f',3) +
													   QString("  ") +
													   QString::number(finalTransformation(2,2),'f',3) +
													   QString("  ") +
													   QString::number(finalTransformation(2,3),'f',3) +
													   QString("\n  ") +
													   QString::number(finalTransformation(3,0),'f',3) +
													   QString("  ") +
													   QString::number(finalTransformation(3,1),'f',3) +
													   QString("  ") +
													   QString::number(finalTransformation(3,2),'f',3) +
													   QString("  ") +
													   QString::number(finalTransformation(3,3),'f',3) +
													   QString("  ]"));
				// Angles
				angleX = atan2(finalTransformation(2,1),finalTransformation(2,2));
				ui->txtAngleX->appendPlainText(QString::number(angleX,'f',3));
				angleY = asin(-finalTransformation(2,0));
				ui->txtAngleY->appendPlainText(QString::number(angleY,'f',3));
				angleZ = atan2(finalTransformation(1,0),finalTransformation(0,0));
				ui->txtAngleZ->appendPlainText(QString::number(angleZ,'f',3));
				// Translation
				ui->txtTx->appendPlainText(QString::number(finalTransformation(0,3),'f',3));
				ui->txtTy->appendPlainText(QString::number(finalTransformation(1,3),'f',3));
				ui->txtTz->appendPlainText(QString::number(finalTransformation(2,3),'f',3));
				// Re-Init transformation matrix
				finalTransformation = Eigen::Matrix4f::Identity();
				iterations = 0;
				registerClouds = false;
			}
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

void KinectViewer::on_btnAddCloud_clicked()
{
    addCloud = true;
}

void KinectViewer::on_btnRemoveCloud_clicked()
{
    removeClouds = true;
}

void KinectViewer::on_btnRegistration_clicked()
{
    registerClouds = true;
}
