#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Initialize
    initialize();

    // Set up the QVTK window
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());

    // Initialize camera position
    viewer->setCameraPosition(0.0, 0.0, -1.0, 0.0, 0.0, 0.0);

    // Add Coordinate System
    viewer->addCoordinateSystem(0.01);

    // Timer for 3D/UI update
    tmrTimer = new QTimer(this);
    connect(tmrTimer,SIGNAL(timeout()),this,SLOT(updateGUI()));
    tmrTimer->start(); // msec
}

MainWindow::~MainWindow()
{
    delete ui;

    // Finalize
    finalize();
}

// Initialize
void MainWindow::initialize()
{
    // Initialize Sensor
    initializeSensor();

    // Initialize Depth
    initializeDepth();

    // Initialize Point Cloud
    initializePointCloud();
}

// Initialize Sensor
void MainWindow::initializeSensor()
{
    // Open Sensor
    GetDefaultKinectSensor(&kinect);
    kinect->Open();

    // Check Open
    BOOLEAN isOpen = FALSE;
    kinect->get_IsOpen(&isOpen);
    if( !isOpen ){
        throw std::runtime_error("failed IKinectSensor::get_IsOpen( &isOpen )");
    }

    // Retrieve Coordinate Mapper
    kinect->get_CoordinateMapper(&coordinateMapper);
}

// Initialize Depth
void MainWindow::initializeDepth()
{
    // Open Depth Reader
    ComPtr<IDepthFrameSource> depthFrameSource;
    kinect->get_DepthFrameSource(&depthFrameSource);
    depthFrameSource->OpenReader(&depthFrameReader);

    // Retrieve Depth Description
    ComPtr<IFrameDescription> depthFrameDescription;
    depthFrameSource->get_FrameDescription(&depthFrameDescription);
    depthFrameDescription->get_Width(&depthWidth); // 512
    depthFrameDescription->get_Height(&depthHeight); // 424
    depthFrameDescription->get_BytesPerPixel(&depthBytesPerPixel); // 2

    // Allocation Depth Buffer
    //depthBuffer.resize(depthWidth * depthHeight);
}

// Initialize Point Cloud
void MainWindow::initializePointCloud()
{
    // Create Point Cloud
    //cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = static_cast<uint32_t>(depthWidth);
    cloud->height = static_cast<uint32_t>( depthHeight);
    cloud->points.resize(cloud->height * cloud->width);
    cloud->is_dense = false;
}

// Processing
void MainWindow::updateGUI() {

    // Update Color
    //updateColor();

    // Update Depth
    updateDepth();

    // Update Point Cloud
    //updatePointCloud();

    // Prepare Point Cloud
    preparePointCloud();

    // Draw Point Cloud
    drawPointCloud();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MainWindow::convertDepthToPointXYZ(UINT16* depthBuffer)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    cloud->width = static_cast<uint32_t>( depthWidth );
    cloud->height = static_cast<uint32_t>( depthHeight );
    cloud->is_dense = false;

    cloud->points.resize( cloud->height * cloud->width );

    pcl::PointXYZ* pt = &cloud->points[0];
    for( int y = 0; y < depthHeight; y++ ){
        for( int x = 0; x < depthWidth; x++, pt++ ){
            pcl::PointXYZ point;

            DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y)};
            UINT16 depth = depthBuffer[y * depthWidth + x];

            // Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
            CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
            coordinateMapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint );
            point.x = cameraSpacePoint.X;
            point.y = cameraSpacePoint.Y;
            point.z = cameraSpacePoint.Z;

            *pt = point;
        }
    }

    return cloud;
}

// Update Color
void MainWindow::updateColor()
{
    // Retrieve Color Frame
    ComPtr<IColorFrame> colorFrame;
    const HRESULT ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
    if( FAILED(ret)){
        return;
    }

    // Convert Format ( YUY2 -> BGRA )
    ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra));
}

// Update Depth
void MainWindow::updateDepth()
{
    // Retrieve Depth Frame
    ComPtr<IDepthFrame> depthFrame;
    const HRESULT ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
    if(FAILED(ret)){
        return;
    }

    // Retrieve Depth Data
    ERROR_CHECK(depthFrame->AccessUnderlyingBuffer(&nBufferSize, &depthBuffer));
    //ERROR_CHECK(depthFrame->CopyFrameDataToArray(static_cast<UINT>(depthBuffer.size()), &depthBuffer[0]));
}

// Update Point Cloud
void MainWindow::updatePointCloud()
{
//    // Reset Point Cloud
//    cloud->clear();

//    // Convert to Point Cloud
//    for(int depthY = 0; depthY < depthHeight; depthY++){
//        for(int depthX = 0; depthX < depthWidth; depthX++){
//            pcl::PointXYZRGBA point;

//            // Retrieve Mapped Coordinates
//            DepthSpacePoint depthSpacePoint = { static_cast<float>(depthX), static_cast<float>(depthY) };
//            UINT16 depth = depthBuffer[depthY * depthWidth + depthX];
//            ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
//            ERROR_CHECK(coordinateMapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint));

//            // Set Color to Point
//            int colorX = static_cast<int>(colorSpacePoint.X + 0.5f);
//            int colorY = static_cast<int>(colorSpacePoint.Y + 0.5f);
//            if((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)){
//                unsigned int colorIndex = (colorY * colorWidth + colorX) * colorBytesPerPixel;
//                point.b = colorBuffer[colorIndex + 0];
//                point.g = colorBuffer[colorIndex + 1];
//                point.r = colorBuffer[colorIndex + 2];
//                point.a = colorBuffer[colorIndex + 3];
//            }

//            // Retrieve Mapped Coordinates
//            CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
//            ERROR_CHECK(coordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint));

//            // Set Depth to Point
//            if((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)){
//                point.x = cameraSpacePoint.X;
//                point.y = cameraSpacePoint.Y;
//                point.z = cameraSpacePoint.Z;
//            }

//            // Set Point to Point Cloud
//            cloud->push_back(point);
//        }
//    }
}

// Update Point Cloud
void MainWindow::preparePointCloud()
{
    // Reset Point Cloud
    cloud->clear();

    // Convert to Point Cloud
    for(int depthY = 0; depthY < depthHeight; depthY++){
        for(int depthX = 0; depthX < depthWidth; depthX++){
            pcl::PointXYZ point;

            DepthSpacePoint depthSpacePoint = { static_cast<float>(depthX), static_cast<float>(depthY) };
            //UINT16 depth = depthBuffer[depthY * depthWidth + depthX];
            UINT16 depth = depthY * depthWidth + depthX;

//            // Retrieve Mapped Coordinates
//
//
//            ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
//            ERROR_CHECK(coordinateMapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint));

            // Set Color to Point
//            int colorX = static_cast<int>(colorSpacePoint.X + 0.5f);
//            int colorY = static_cast<int>(colorSpacePoint.Y + 0.5f);
//            if((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)){
//                unsigned int colorIndex = (colorY * colorWidth + colorX) * colorBytesPerPixel;
//                point.b = 150.0f; //colorBuffer[colorIndex + 0];
//                point.g = 150.0f; //colorBuffer[colorIndex + 1];
//                point.r = 150.0f; //colorBuffer[colorIndex + 2];
//                point.a = 150.0f; //colorBuffer[colorIndex + 3];
//            }

            // Retrieve Mapped Coordinates
            CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
            ERROR_CHECK(coordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint));

            // Set Depth to Point
            //if((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)){
            point.x = cameraSpacePoint.X;
            point.y = cameraSpacePoint.Y;
            point.z = cameraSpacePoint.Z;
            //}

            // Set Point to Point Cloud
            cloud->push_back(point);
        }
    }

    if (saveCloud)
    {
        std::stringstream stream;
        stream << "inputCloud" << filesSaved << ".pcd";
        std::string filename = stream.str();
        if (pcl::io::savePCDFile(filename, *cloud, true) == 0)
        {
            filesSaved++;
            cout << "Saved " << filename << "." << endl;
        }

        if (filesSaved == 10){
           saveCloud = false;
        }
    }
}

// Draw Point Cloud
void MainWindow::drawPointCloud()
{
    // Update Point Cloud
    if(!viewer->updatePointCloud(cloud, "cloud")){
        viewer->addPointCloud(cloud, "cloud");
    }

    // Update viewer
    ui->qvtkWidget->update();
}

// Finalize
void MainWindow::finalize()
{
    // Close Sensor
    if(kinect != nullptr){
        kinect->Close();
    }
}
