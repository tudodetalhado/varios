#include <fstream>
#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>


static void pcd2obj(const std::string& inputFilename, const std::string& outputFilename)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(inputFilename, cloud) == -1)
	{
		std::cerr << "Couldn't read file " << inputFilename << std::endl;
		return;
	}

	const size_t size = cloud.points.size();
	std::ofstream os(outputFilename.c_str());

	for (unsigned int i = 0; i<size; i++)
	{
		// Remove nan
		if (!std::isnan(cloud.points[i].x))
		{
			os << "v ";
			os << cloud.points[i].x << " ";
			os << cloud.points[i].y << " ";
			os << cloud.points[i].z << "\n";
		}
	}

	os.close();
}

static void obj2pcd(const std::string& inputFilename, const std::string& outputFilename)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Input stream
	std::ifstream is(inputFilename.c_str());

	// Read line by line
	for (std::string line; std::getline(is, line);)
	{
		std::istringstream in(line);

		std::string v;
		in >> v;

		// Read x y z
		float x, y, z;
		in >> x >> y >> z;

		cloud.push_back(pcl::PointXYZ(x, y, z));
	}

	is.close();

	// Save to pcd file
	pcl::io::savePCDFileBinaryCompressed(outputFilename, cloud);
}

int main(int argc, char* argv[])
{
	if (argc < 3)
	{
		std::cerr
			<< std::endl
			<< "<Error> Missing parameters" << std::endl
			<< "Usage:  ConverterPcdObj.exe input.obj output.pcd" << std::endl
			<< "Usage:  ConverterPcdObj.exe input.pcd output.obj" << std::endl;
		return EXIT_FAILURE;
	}


	std::string inputFile = argv[1];
	std::string outputFile = argv[2];

	if (inputFile.find(".pcd") != std::string::npos)
	{
		if (outputFile.find(".obj") != std::string::npos)
		{
			pcd2obj(inputFile, outputFile);
		}
		else if (outputFile.find(".ply") != std::string::npos)
		{
			pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
			pcl::PCDReader reader;
			reader.read(inputFile, *cloud);
			pcl::io::savePLYFile(outputFile, *cloud);
		}
	}
	else if (inputFile.find(".obj") != std::string::npos && outputFile.find(".pcd") != std::string::npos)
	{
		//obj2pcd(inputFile, outputFile);
		pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
		pcl::OBJReader reader;
		reader.read(inputFile, *cloud);
		pcl::PCDWriter writer;
		writer.write(outputFile, *cloud);
	}
	else if (inputFile.find(".ply") != std::string::npos && outputFile.find(".pcd") != std::string::npos)
	{
		pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
		pcl::PLYReader reader;
		reader.read(inputFile, *cloud);
		pcl::PCDWriter writer;
		writer.write(outputFile, *cloud);
	}
	else
	{
		std::cerr
			<< std::endl
			<< "<Error> Could not detect file format." << std::endl
			<< "Usage:  ConverterPcdObj.exe input.obj output.pcd" << std::endl
			<< "Usage:  ConverterPcdObj.exe input.pcd output.obj" << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
