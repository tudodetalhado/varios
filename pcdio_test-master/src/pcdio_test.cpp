#include <cassert>
#include <iostream>
#include <string>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

template <typename T> void vis(T cloud, std::string title) {
  auto viewer = pcl::visualization::PCLVisualizer::Ptr(
      new pcl::visualization::PCLVisualizer(title));
  viewer->addPointCloud(cloud);
  viewer->spin();
}

int main() {
  std::string inputPCDPath = "../pcd/kumamon.pcd";
  std::string outputPCDPath = "../pcd/kumamon_output.pcd";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  assert(pcl::io::loadPCDFile(inputPCDPath, *input_cloud) == 0);

  vis(input_cloud, "before");
  assert(pcl::io::savePCDFile(outputPCDPath, *input_cloud, false) == 0);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  assert(pcl::io::loadPCDFile(outputPCDPath, *output_cloud) == 0);

  vis(output_cloud, "after");

  return 0;
}
