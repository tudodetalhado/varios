#define PCL_NO_PRECOMPILE	
// Workaround for a problem: http://www.pcl-users.org/PCL-Features-Library-Limit-Exceeded-td4026817.html
#define CLOUD_SIZE 100000
#define ITERATION 100
#define FILE_NAME_ASCII "cloud_ascii.pcd"
#define FILE_NAME_BIN "cloud_bin.pcd"
#define FILE_NAME_BIN_CON "cloud_bin_con.pcd"

#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef PointCloudT::Ptr PointCloudPtrT;
typedef PointCloudT::ConstPtr PointCloudConstPtrT;

void createRandomPointCloud(PointCloudT &cloud, unsigned int size) {
    cloud.resize(size);
    BOOST_FOREACH(PointT &p, cloud) { p.getVector3fMap() = Eigen::Vector3f::Random(); }
}

double getFileSize(const std::string &file_path) {
    double file_size(-1.0);
    try {
        file_size = boost::filesystem::file_size(boost::filesystem::path(file_path));
    } catch(boost::filesystem::filesystem_error &ex) {
        std::cerr << ex.what() << std::endl;
    }
    return file_size;
}

double getFileSizeKB(const std::string &file_path) { return getFileSize(file_path) / 1024; }
double getFileSizeMB(const std::string &file_path) { return getFileSizeKB(file_path) / 1024; }

void main() {
    pcl::console::TicToc tt;
    std::printf("Cloud size: %d\n", CLOUD_SIZE);
    std::printf("Iteration : %d\n\n", ITERATION);

    // Create data
    PointCloudPtrT cloud(new PointCloudT);
    createRandomPointCloud(*cloud, CLOUD_SIZE);

    // Save as ASCII
    std::printf("---- Save ----\n");

    std::printf("ASCII : ");
    tt.tic();
    for(int i = 0; i < ITERATION; i++) { pcl::io::savePCDFileASCII(FILE_NAME_ASCII, *cloud); }
    std::printf("average %lf ms, %lf MB\n", tt.toc() / ITERATION, getFileSizeMB(FILE_NAME_ASCII));

    std::printf("Binary: ");
    tt.tic();
    for(int i = 0; i < ITERATION; i++) { pcl::io::savePCDFileBinary(FILE_NAME_BIN, *cloud); }
    std::printf("average %lf ms, %lf MB\n", tt.toc() / ITERATION, getFileSizeMB(FILE_NAME_BIN));

    std::printf("BinCon: ");
    tt.tic();
    for(int i = 0; i < ITERATION; i++) { pcl::io::savePCDFileBinaryCompressed(FILE_NAME_BIN_CON, *cloud); }
    std::printf("average %lf ms, %lf MB\n\n", tt.toc() / ITERATION, getFileSizeMB(FILE_NAME_BIN_CON));

    // Save as ASCII
    std::printf("---- Load ----\n");

    std::printf("ASCII : ");
    tt.tic();
    for(int i = 0; i < ITERATION; i++) { pcl::io::loadPCDFile(FILE_NAME_ASCII, *cloud); }
    std::printf("average %lf ms\n", tt.toc() / ITERATION);

    std::printf("Binary: ");
    tt.tic();
    for(int i = 0; i < ITERATION; i++) { pcl::io::loadPCDFile(FILE_NAME_BIN, *cloud); }
    std::printf("average %lf ms\n", tt.toc() / ITERATION);

    std::printf("BinCon: ");
    tt.tic();
    for(int i = 0; i < ITERATION; i++) { pcl::io::loadPCDFile(FILE_NAME_BIN_CON, *cloud); }
    std::printf("average %lf ms\n\n", tt.toc() / ITERATION);

    std:system("pause");
}
