#ifndef DEPTHBASICS_H
#define DEPTHBASICS_H

#include <iostream>
#include "kinectsensor.h"

// Qt
#include <QTimer>
#include <QMainWindow>

// Point Cloud Library
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

namespace Ui {
class CDepthBasics;
}

class CDepthBasics : public QMainWindow
{
    Q_OBJECT

public:
    explicit CDepthBasics(QWidget *parent = 0);
    ~CDepthBasics();

public slots:
  void updateView();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

private:
    Ui::CDepthBasics *ui;
    KinectSensor *kinect;
    QTimer *tmrTimer;
};

#endif // DEPTHBASICS_H
