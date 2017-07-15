#include "DepthBasics.h"
#include "ui_DepthBasics.h"

CDepthBasics::CDepthBasics(QWidget *parent) :
    QMainWindow(parent), ui(new Ui::CDepthBasics)
{
    ui->setupUi(this);

    // Kinect
    kinect = new KinectSensor();

    // Timer for 3D/UI update
    tmrTimer = new QTimer(this);
    connect(tmrTimer, SIGNAL(timeout()), this, SLOT(updateView()));
    tmrTimer->start(); // msec

    // Set up the QVTK window
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    // Initialize camera position
    viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, 0.0, 0.0);

    // Add Coordinate System
    viewer->addCoordinateSystem(0.1);
}

CDepthBasics::~CDepthBasics()
{
    delete ui;
}

// Processing
void CDepthBasics::updateView() {

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = kinect->getPCLCloud();

    // Update Point Cloud
    if(!viewer->updatePointCloud(cloud, "cloud")){
        viewer->addPointCloud(cloud, "cloud");
    }

    // Update viewer
    ui->qvtkWidget->update();
}
