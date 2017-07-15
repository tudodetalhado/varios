
#include <SDKDDKVer.h>

#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;


#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

// Converts degrees to radians.
#define DegToRad(angle_degrees) (angle_degrees * M_PI / 180.0)

// Converts radians to degrees.
#define RadToDeg(angle_radians) (angle_radians * 180.0 / M_PI)


namespace pcl
{		
	TEST_CLASS(icp)
	{
	public:
		
		TEST_METHOD(TestCupCloud)
		{
			typedef pcl::PointXYZRGB PointT;
			typedef pcl::PointCloud<PointT> PointCloudT;
			PointCloudT::Ptr cloud_input(new PointCloudT);
			PointCloudT::Ptr cloud_transformed(new PointCloudT);
			const std::string input_file("../../data/cup.pcd");
			const std::string output_folder("../../data/");

			Assert::IsTrue(pcl::io::loadPCDFile(input_file, *cloud_input) > -1, L"\n<TestCupCloud could not load input file>\n", LINE_INFO());


			for (int i = 0; i < 10; ++i)
			{
				Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
				double theta = (i * M_PI) / 180.0;
				transformation_matrix(0, 0) = cos(theta);
				transformation_matrix(0, 1) = -sin(theta);
				transformation_matrix(1, 0) = sin(theta);
				transformation_matrix(1, 1) = cos(theta);

				transformation_matrix(0, 3) = 0.05;	// A translation on X axis (0.02 meters)
				transformation_matrix(2, 3) = 0.05;	// A translation on Z axis (0.02 meters)

				pcl::transformPointCloud(*cloud_input, *cloud_transformed, transformation_matrix);

				std::stringstream filename;
				filename << output_folder << i << ".pcd";

				pcl::PCDWriter w;
				Assert::IsTrue(w.writeBinaryCompressed(filename.str(), *cloud_transformed) > -1, L"\n<TestCupCloud could not save output file>\n", LINE_INFO());
			}
		}


		TEST_METHOD(TestRotateCloud)
		{
			const std::string filepath = "../../data/thai_lion/thai_lion.pcd";
			const std::string basename = "thai_lion";

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);  // point cloud transformed
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);  // point cloud transformed

			Assert::IsTrue(pcl::io::loadPCDFile(filepath, *cloud_input) > -1, L"\n<TestRotateCloud could not load input file>\n", LINE_INFO());

			pcl::PCDWriter w;
			Eigen::Matrix4d transformation_matrix;
			Eigen::Affine3d rotation;

			// save original
			w.writeBinaryCompressed(basename + "_00.pcd", *cloud_input);

			// 15º
			rotation = Eigen::Affine3d::Identity();
			rotation.rotate(Eigen::AngleAxisd(DegToRad(15.0), Eigen::Vector3d::UnitY()));
			transformation_matrix.block(0, 0, 3, 3) = rotation.matrix();
			pcl::transformPointCloud(*cloud_input, *cloud_output, transformation_matrix);
			// save
			w.writeBinaryCompressed(basename + "_15.pcd", *cloud_output);


			// 30º
			rotation = Eigen::Affine3d::Identity();
			rotation.rotate(Eigen::AngleAxisd(DegToRad(30.0), Eigen::Vector3d::UnitY()));
			transformation_matrix.block(0, 0, 3, 3) = rotation.matrix();
			pcl::transformPointCloud(*cloud_input, *cloud_output, transformation_matrix);
			//save
			w.writeBinaryCompressed(basename + "_30.pcd", *cloud_output);


			// 45º
			rotation = Eigen::Affine3d::Identity();
			rotation.rotate(Eigen::AngleAxisd(DegToRad(45.0), Eigen::Vector3d::UnitY()));
			transformation_matrix.block(0, 0, 3, 3) = rotation.matrix();
			pcl::transformPointCloud(*cloud_input, *cloud_output, transformation_matrix);
			//save
			w.writeBinaryCompressed(basename + "_45.pcd", *cloud_output);


			// 60º
			rotation = Eigen::Affine3d::Identity();
			rotation.rotate(Eigen::AngleAxisd(DegToRad(60.0), Eigen::Vector3d::UnitY()));
			transformation_matrix.block(0, 0, 3, 3) = rotation.matrix();
			pcl::transformPointCloud(*cloud_input, *cloud_output, transformation_matrix);
			//save
			w.writeBinaryCompressed(basename + "_60.pcd", *cloud_output);


			// 75º
			rotation = Eigen::Affine3d::Identity();
			rotation.rotate(Eigen::AngleAxisd(DegToRad(75.0), Eigen::Vector3d::UnitY()));
			transformation_matrix.block(0, 0, 3, 3) = rotation.matrix();
			pcl::transformPointCloud(*cloud_input, *cloud_output, transformation_matrix);
			//save
			w.writeBinaryCompressed(basename + "_75.pcd", *cloud_output);

			// 90º
			rotation = Eigen::Affine3d::Identity();
			rotation.rotate(Eigen::AngleAxisd(DegToRad(90.0), Eigen::Vector3d::UnitY()));
			transformation_matrix.block(0, 0, 3, 3) = rotation.matrix();
			pcl::transformPointCloud(*cloud_input, *cloud_output, transformation_matrix);
			//save
			w.writeBinaryCompressed(basename + "_90.pcd", *cloud_output);
			
		}

	};
}