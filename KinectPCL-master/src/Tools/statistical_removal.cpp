
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/time.h>   // TicToc

int main(int argc, char** argv)
{
	if (argc < 3)
	{
		std::cerr
			<< "Missing parameters. Abort."
			<< "Usage: statistical_removal.exe <point_cloud.pcd> <MeanK (int)> <StddevThreashold (float)> "
			<< std::endl;
		return EXIT_FAILURE;
	}
#if 1
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
	PointCloudT::Ptr cloud(new PointCloudT);
	PointCloudT::Ptr cloud_filtered(new PointCloudT);



	pcl::io::loadPCDFile(argv[1], *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	pcl::console::TicToc time;
	time.tic();

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(atoi(argv[2]));
	sor.setStddevMulThresh(atof(argv[3]));
	sor.filter(*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	std::cout << "Time spent : " << time.toc() << " ms" << std::endl;

	pcl::PCDWriter w;
	w.writeBinaryCompressed("output_filtered.pcd", *cloud_filtered);

	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	w.writeBinaryCompressed("output_filtered_negative.pcd", *cloud_filtered);
#else
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	
	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZ>(argv[1], *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	pcl::console::TicToc time;
	time.tic();

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(atoi(argv[2]));
	sor.setStddevMulThresh(atof(argv[3]));
	sor.filter(*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	std::cout << "Time spent : " << time.toc() << " ms" << std::endl;


	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("output_filtered.pcd", *cloud_filtered, false);

	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZ>("output_filtered_negative.pcd", *cloud_filtered, false);
#endif
	return (0);
}