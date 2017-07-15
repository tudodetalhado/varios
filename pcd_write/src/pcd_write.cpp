//Complete Viewer – C++ program to create your dataset
//Complete Viewer is a C++ program used to save all the streams provided by the Kinect V2.
http://www.tlc.dii.univpm.it/blog/databases4kinect

//If you use the program, please cite the following paper:
E. Cippitelli, S. Gasparrini, S. Spinsante, E. Gambi, “Kinect as a Tool for Gait Analysis: Validation of a Real Time Joints Extraction Algorithm Working in Side View,” . Sensors 2015, 15, 1417-1434. Open Access, available in:
http://www.mdpi.com/1424-8220/15/1/1417

//Project Owl: Kinect V2 Point Cloud Generator for Grasshopper
https://hojoongchung.wordpress.com/2015/09/16/project-owl-kinect-v2-point-cloud-generator-for-grasshopper/


//Save Kinect v2 color stream at 30fps into .bin file

I need to save depth stream and color stream from Kinect 2v at same time. But the frame rate is so high that 
there is a heavy delay and many frames cannot be saved. Is it possible to save both of them? How to improve?

//Save Kinect color frames to a video file

//Have you tried ffmpeg?

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 300000;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}

/**/
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_write pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})
/**/



 void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
 {
   if (!viewer.wasStopped())
   {
       viewer.showCloud (cloud);
	   //Note: you may also use
       pcl::io::savePCDFile("test_pcd.pcd", cloud, true);
	   //to save in binary mode that is a lot faster (but you may not read the file in text editors)
	   //you mean pcl::io::savePCDFileBinary to save in binary mode, right?
   }
 }
 
//The sample project called "Depth with Color-D3D" that is available in the Kinect for Windows Developer Toolkit v1.8.0 is great! Is there anything similar for Kinect v2 available?

//Take a look at the Coordinate Mapping Basics-D2D sample. It demonstrates a green-screen type effect.


//C++11 introduced a lot of goodies like std::mutex (and std::thread too, 
//so that can help eliminate some Qt-specific threading code)
#include <fstream>   
#include <mutex>
#include <thread>    
#include <vector>

std::mutex m;
std::ofstream file;

int main() {
  file.open("file.txt");

  std::vector<std::thread> workers;
  for (int i = 0; i < 10; ++i) {       
    workers.push_back(std::thread([=i] {
      for (int j = 0; j < 10; ++j) {
        std::lock_guard<std::mutex> lock(m);
        file << "thread " << i << ": " << j << endl;
      }
    }));
  }
  for (auto& worker : workers) {
    worker.join();
  }

  file.close();
  return 0;
}

//Example (write binary data to a stream):
QFile file = new QFile("file.dat");
QFile.OpenMode mode = new QFile.OpenMode();
mode.set(QFile.OpenModeFlag.WriteOnly);
file.open(mode);
QDataStream out = new QDataStream(file);   // we will serialize the data into the file
out.writeString("the answer is");   // serialize a string
out.writeInt(42);        // serialize an integer

//Example (read binary data from a stream):
QFile file = new QFile("file.dat");
QFile.OpenMode mode = new QFile.OpenMode();
mode.set(QFile.OpenModeFlag.ReadOnly);
file.open(mode);
QDataStream in = new QDataStream(file);    // read the data serialized from the file
// extract "the answer is" and 42
String str = in.readString();
int a = in.readInt();

//Writing a binary file in C++ very fast
//I just timed 8GB in 36sec, which is about 220MB/s and I think that maxes out my SSD
#include <stdio.h>
const unsigned long long size = 8ULL*1024ULL*1024ULL;
unsigned long long a[size];

int main()
{
    FILE* pFile;
    pFile = fopen("file.binary", "wb");
    for (unsigned long long j = 0; j < 1024; ++j){
        //Some calculations to fill a[]
        fwrite(a, 1, size*sizeof(unsigned long long), pFile);
    }
    fclose(pFile);
    return 0;
}
