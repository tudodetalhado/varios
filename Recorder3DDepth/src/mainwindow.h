#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <sstream>
#include <stdexcept>

#define NOMINMAX

// Qt
#include <QTimer>
#include <QString>
#include <QFileDialog>
#include <QMainWindow>

// Kinect
#include <Kinect.h>

// Point Cloud Library
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include <wrl/client.h>
using namespace Microsoft::WRL;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

  pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ(UINT16* depthBuffer);

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::thread thread;

public slots:
  void updateGUI();
  void onOpen();

private:
    Ui::MainWindow *ui;
    QTimer *tmrTimer;

    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Coordinate Mapper
    ComPtr<ICoordinateMapper> coordinateMapper;

    // Reader
    ComPtr<IColorFrameReader> colorFrameReader;
    ComPtr<IDepthFrameReader> depthFrameReader;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;

    // Depth Buffer
    UINT nBufferSize = 0;
    UINT16 *depthBuffer = NULL;
    //std::vector<UINT16> depthBuffer;
    int depthWidth;
    int depthHeight;
    unsigned int depthBytesPerPixel;

    // PCL
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    void initialize();
    void initializeSensor();
    void initializeColor();
    void initializeDepth();
    void initializePointCloud();
    void updateColor();
    void updateDepth();
    void updatePointCloud();
    void drawPointCloud();
    void finalize();
    void preparePointCloud();
};

#endif // MAINWINDOW_H
