#include <iostream>
#include <string>
#include <fstream>
#include <Eigen/Dense>
#include <vector>


#if 0
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

#include <pcl/filters/voxel_grid.h>
#endif


#define DegToRad(angle_degrees) (angle_degrees * M_PI / 180.0)		// Converts degrees to radians.
#define RadToDeg(angle_radians) (angle_radians * 180.0 / M_PI)		// Converts radians to degrees.


#define MinTruncation 1.0
#define MaxTruncation 1.0
#define MaxWeight 3.0


const double fov_y = 70.0;
float window_width = 512.0f;
float window_height = 424.0f;
float near_plane = 0.1f; // 0.1f;
float far_plane = 100.0f; // 10240.0f;
Eigen::MatrixXd									K = Eigen::MatrixXd(3, 4);
Eigen::Matrix4d									K_proj = Eigen::Matrix4d::Zero();
std::pair<Eigen::MatrixXd, Eigen::MatrixXd>		Rt(Eigen::MatrixXd(3, 4), Eigen::MatrixXd(3, 4));
std::pair<Eigen::Matrix3d, Eigen::Matrix3d>		R(Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero());
std::pair<Eigen::Vector3d, Eigen::Vector3d>		t;
std::pair<Eigen::Matrix4d, Eigen::Matrix4d>		T(Eigen::Matrix4d::Zero(), Eigen::Matrix4d::Zero());

template<typename Type>
struct Voxel
{
	Eigen::Matrix<Type, 3, 1> point;
	Type tsdf;
	Type weight;

	Voxel() :tsdf(FLT_MAX), weight(0.0){}
};
typedef Voxel<double> Voxeld;
typedef Voxel<float> Voxelf;



template<typename T>
static void vector_write(std::ostream& out_file, const std::vector<T>& data)
{
	const std::size_t count = data.size();
	out_file.write(reinterpret_cast<const char*>(&count), sizeof(std::size_t));
	out_file.write(reinterpret_cast<const char*>(&data[0]), count * sizeof(T));
}

template<typename T>
static void vector_read(std::istream& in_file, std::vector<T>& data)
{
	std::size_t count;
	in_file.read(reinterpret_cast<char*>(&count), sizeof(std::size_t));
	data.resize(count);
	in_file.read(reinterpret_cast<char*>(&data[0]), count * sizeof(T));
}

struct KinectFrame
{
	// info : [0] color_width
	// info : [1] color_height
	// info : [2] color_channels
	// info : [3] depth_width
	// info : [4] depth_width
	// info : [5] depth_min_distance
	// info : [6] depth_max_distance
	enum
	{
		ColorWidth = 0,
		ColorHeight,
		ColorChannels,
		DepthWidth,
		DepthHeight,
		DepthMinDistance,
		DepthMaxDistance,
		eTotal
	};

	KinectFrame()
	{
		reset();
	}
	KinectFrame(const std::vector<unsigned short>& _info,
		const std::vector<unsigned char>& _color,
		const std::vector<unsigned short>& _depth) :
		info(_info), color(_color), depth(_depth){}


	void clear()
	{
		info.clear();
		color.clear();
		depth.clear();
	}

	void reset()
	{
		info.resize(eTotal, 0);
		color.resize(1920 * 1080 * 4, 0);
		depth.resize(512 * 424, 0);
	}

	static void load(const std::string& filename, KinectFrame& frame)
	{
		std::ifstream in_file;
		in_file.open(filename, std::ios::in | std::ios::binary);
		vector_read(in_file, frame.info);
		vector_read(in_file, frame.color);
		vector_read(in_file, frame.depth);
		in_file.close();
	}

	//static void save(std::string filename, std::vector<unsigned short> info, std::vector<unsigned char> color_buffer, std::vector<unsigned short> depth_buffer)
	static void save(const std::string& filename, const KinectFrame& frame)
	{
		std::ofstream out;
		out.open(filename, std::ofstream::out | std::ofstream::binary);
		vector_write(out, frame.info);
		vector_write(out, frame.color);
		vector_write(out, frame.depth);
		out.close();
	}

	unsigned short color_width() const { return info[ColorWidth]; }
	unsigned short color_height() const { return info[ColorHeight]; }
	unsigned short depth_width() const { return info[DepthWidth]; }
	unsigned short depth_height() const { return info[DepthHeight]; }
	unsigned short depth_min_distance() const { return info[DepthMinDistance]; }
	unsigned short depth_max_distance() const { return info[DepthMaxDistance]; }
	unsigned short color_at(int x, int y)const { return color.at(y * info[ColorWidth] + x); }
	unsigned short depth_at(int x, int y)const { return depth.at(y * info[DepthWidth] + x); }

	std::vector<unsigned short> info;
	std::vector<unsigned char> color;
	std::vector<unsigned short> depth;
};


static Eigen::Matrix4d perspective_matrix(double fovy, double aspect_ratio, double near_plane, double far_plane)
{
	Eigen::Matrix4d out = Eigen::Matrix4d::Zero();

	const double	y_scale = 1.0 / tan((fovy / 2.0)*(M_PI / 180.0));
	const double	x_scale = y_scale / aspect_ratio;
	const double	depth_length = far_plane - near_plane;

	out(0, 0) = x_scale;
	out(1, 1) = y_scale;
	out(2, 2) = -((far_plane + near_plane) / depth_length);
	out(3, 2) = -1.0;
	out(2, 3) = -((2 * near_plane * far_plane) / depth_length);

	return out;
}


static Eigen::Matrix4d perspective_matrix_inverse(double fovy, double aspect_ratio, double near_plane, double far_plane)
{
	Eigen::Matrix4d out = Eigen::Matrix4d::Zero();

	const double	y_scale = 1.0 / tan((fovy / 2.0)*(M_PI / 180.0));
	const double	x_scale = y_scale / aspect_ratio;
	const double	depth_length = far_plane - near_plane;

	out(0, 0) = 1.0 / x_scale;
	out(1, 1) = 1.0 / y_scale;
	out(2, 3) = -1.0;
	out(3, 2) = -1.0 / ((2 * near_plane * far_plane) / depth_length);
	out(3, 3) = ((far_plane + near_plane) / depth_length) / ((2 * near_plane * far_plane) / depth_length);

	return out;
}



Eigen::Vector3d vertex_to_window_coord(Eigen::Vector4d p3d, double fovy, double aspect_ratio, double near_plane, double far_plane, int window_width, int window_height)
{
	const Eigen::Matrix4d proj = perspective_matrix(fovy, aspect_ratio, near_plane, far_plane);

	const Eigen::Vector4d p_clip = proj * p3d;

	const Eigen::Vector3d p_ndc = (p_clip / p_clip.w()).head<3>();

	Eigen::Vector3d p_window;
	p_window.x() = window_width / 2.0 * p_ndc.x() + window_width / 2.0;
	p_window.y() = window_height / 2.0 * p_ndc.y() + window_height / 2.0;
	p_window.z() = (far_plane - near_plane) / 2.0 * p_ndc.z() + (far_plane + near_plane) / 2.0;

	return p_window;
}

Eigen::Vector3d window_coord_to_3d(Eigen::Vector2d pixel, double depth, double fovy, double aspect_ratio, double near_plane, double far_plane, int window_width, int window_height)
{
	Eigen::Vector3d ndc;
	ndc.x() = (pixel.x() - (window_width / 2.0)) / (window_width / 2.0);
	ndc.y() = (pixel.y() - (window_height / 2.0)) / (window_height / 2.0);
	ndc.z() = -1.0f;

	const Eigen::Vector3d clip = ndc * depth;

	const Eigen::Matrix4d proj_inv = perspective_matrix_inverse(fovy, aspect_ratio, near_plane, far_plane);
	const Eigen::Vector4d vertex_proj_inv = proj_inv * clip.homogeneous();

	Eigen::Vector3d p3d_final;
	p3d_final.x() = -vertex_proj_inv.x();
	p3d_final.y() = -vertex_proj_inv.y();
	p3d_final.z() = depth;

	return p3d_final;
}


void setupMatrices()
{
	//
	// K matrix
	//
	//double width = 1936 * 2; // 1920.0;
	//double height = 1296 * 2; // 1080.0;
#if 0
	double width = 512.0;
	double height = 424.0;
	double aspect = 1.0; // width / height;
	//double f = 114.873 / 0.0130887; // 1.0 / std::tan(DegToRad(fov_y) / 2.0);
	//double f = 1.0 / std::tan(DegToRad(fov_y) / 2.0);
	double f = 0.0114;
	K.setZero();
	K(0, 0) = f / aspect;
	K(1, 1) = f;
	K(0, 2) = width / 2.0;
	K(1, 2) = height / 2.0;
	K(2, 2) = 1.0;
#else

#define NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X (0.72113f)
#define NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y (0.870799f)
#define NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X (0.50602675f)
#define NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y (0.499133f)
#define NUI_FUSION_DEFAULT_MINIMUM_DEPTH (0.5f)
#define NUI_FUSION_DEFAULT_MAXIMUM_DEPTH (8.0f)

	//K = perspectiveMatrix(70.0f, 512.f / 424.f, 0.1f, 10240.0f);

	float aspect_ratio = window_width / window_height;
	float y_scale = (float)1.0 / (float)tan((fov_y / 2.0)*(M_PI / 180.0));
	float x_scale = y_scale / aspect_ratio;

	K.setZero();
	K(0, 0) = x_scale;
	K(1, 1) = y_scale;
	K(2, 2) = 1;

	K_proj = perspective_matrix(fov_y, aspect_ratio, near_plane, far_plane);

#endif


	//
	// R matrix
	//
	Eigen::Affine3d rotation = Eigen::Affine3d::Identity();
	rotation.translate(Eigen::Vector3d(0, 0, -105));
	T.first = rotation.matrix();
	rotation.rotate(Eigen::AngleAxisd(DegToRad(90.0), Eigen::Vector3d::UnitY()));		// 90º
	T.second = rotation.matrix();

#if 0
	R.first.setIdentity();
	R.second = rotation.matrix().block(0, 0, 3, 3);

	//
	// t vector
	//
	t.first << 0.0, 0.0, 4000.0;
	t.second << 0.0, 0.0, 4000.0;

	//
	// Rt matrix
	//
	Rt.first.block(0, 0, 3, 3) = R.first;
	Rt.second.block(0, 0, 3, 3) = R.second;
	Rt.first.col(3) = R.first * t.first;
	Rt.second.col(3) = R.second * t.second;

	T.first.block(0, 0, 3, 4) = Rt.first;
	T.second.block(0, 0, 3, 4) = Rt.second;
	T.first.row(3) << 0.0, 0.0, 0.0, 1.0;
	T.second.row(3) << 0.0, 0.0, 0.0, 1.0;
#else

	


#endif

	//T.second = T.first;

	//std::cout << std::fixed << std::endl
	//	<< "Rt: " << std::endl << Rt.second << std::endl
	//	<< "T: " << std::endl << T.second << std::endl;
}


static bool import_obj(const std::string& filename, std::vector<Eigen::Vector3d>& points3D, int max_point_count = INT_MAX)
{
	std::ifstream inFile;
	inFile.open(filename);

	if (!inFile.is_open())
	{
		std::cerr << "Error: Could not open obj input file: " << filename << std::endl;
		return false;
	}

	points3D.clear();

	int i = 0;
	while (inFile)
	{
		std::string str;

		if (!std::getline(inFile, str))
		{
			if (inFile.eof())
				return true;

			std::cerr << "Error: Problems when reading obj file: " << filename << std::endl;
			return false;
		}

		if (str[0] == 'v')
		{
			std::stringstream ss(str);
			std::vector <std::string> record;

			char c;
			double x, y, z;
			ss >> c >> x >> y >> z;

			Eigen::Vector3d p(x, y, z);
			points3D.push_back(p);
		}

		if (i++ > max_point_count)
			break;
	}

	inFile.close();
	return true;
}
static void export_obj(const std::string& filename, const std::vector<Eigen::Vector3d>& points3D)
{
	std::ofstream file;
	file.open(filename);
	for (const auto X : points3D)
	{
		file << std::fixed << "v " << X.transpose() << std::endl;
	}
	file.close();
}

static bool import_xyzd(const std::string& filename, std::vector<Eigen::Vector4d>& points_xyzd, int max_point_count = INT_MAX)
{
	std::ifstream inFile;
	inFile.open(filename);

	if (!inFile.is_open())
	{
		std::cerr << "Error: Could not open obj input file: " << filename << std::endl;
		return false;
	}

	points_xyzd.clear();

	int i = 0;
	while (inFile)
	{
		std::string str;

		if (!std::getline(inFile, str))
		{
			if (inFile.eof())
				return true;

			std::cerr << "Error: Problems when reading obj file: " << filename << std::endl;
			return false;
		}

		if (str[0] == 'v')
		{
			std::stringstream ss(str);
			std::vector <std::string> record;

			char c;
			double x, y, z, d;
			ss >> c >> x >> y >> z >> d;

			Eigen::Vector4d p(x, y, z, d);
			points_xyzd.push_back(p);
		}

		if (i++ > max_point_count)
			break;
	}

	inFile.close();
	return true;
}

static void export_xyzd(const std::string& filename, const std::vector<Eigen::Vector4d>& points_xyzd)
{
	std::ofstream file;
	file.open(filename);
	for (const auto p : points_xyzd)
	{
		file << std::fixed << "v " << p.transpose() << std::endl;
	}
	file.close();
}


static void export_volume(const std::string& filename, const std::vector<Voxeld>& volume, const Eigen::Matrix4d& transformation = Eigen::Matrix4d::Identity())
{
	Eigen::Affine3d rotation;
	Eigen::Vector4d rgb;
	std::ofstream file;
	file.open(filename);
	for (const auto v : volume)
	{
		//rotation = Eigen::Affine3d::Identity();
		//rotation.rotate(Eigen::AngleAxisd(DegToRad(v.tsdf * 180.0), Eigen::Vector3d::UnitZ()));		// 90º
		//rgb = rotation.matrix() * (-Eigen::Vector4d::UnitX());
		
		Eigen::Vector3d rgb(1.0 - v.tsdf, 1.0 - v.tsdf, v.tsdf);
		
		//if (v.tsdf > -0.1 && v.tsdf < 0.1)
			file << std::fixed << "v " << (transformation * v.point.homogeneous()).transpose() << ' ' << rgb.normalized().transpose() << std::endl;
	}
	file.close();
}



int main(int argc, char* argv[])
{
	const std::string filepath = argv[1];
	int vol_size = atoi(argv[2]);
	int vx_size = atoi(argv[3]);
	Eigen::Vector3i voxel_size(vx_size, vx_size, vx_size);
	Eigen::Vector3i volume_size(vol_size, vol_size, vol_size);
	Eigen::Vector3i voxel_count(volume_size.x() / voxel_size.x(), volume_size.y() / voxel_size.y(), volume_size.z() / voxel_size.z());

	//std::cout << std::endl
	//	<< "Volume Size : " << volume_size.transpose() << std::endl
	//	<< "Voxel Size  : " << voxel_size.transpose() << std::endl
	//	<< "Voxel Count : " << voxel_count.transpose() << std::endl << std::endl;

	setupMatrices();

	std::vector<Eigen::Vector3d> points3D;
	import_obj(filepath, points3D);

	std::vector<double> depth_buffer(int(window_width * window_height), -1.0);

	std::cout 
		<< T.first.rows() << ", " << T.first.matrix().cols()
		<< std::endl
		<< T.first << std::endl;


	for (Eigen::Vector3d p3d : points3D)
	{
		Eigen::Vector4d v = T.first * p3d.homogeneous();
		Eigen::Vector3d pixel = vertex_to_window_coord(v, fov_y, window_width / window_height, near_plane, far_plane, (int)window_width, (int)window_height);
		if (pixel.x() > 0 && pixel.x() < window_width &&
			pixel.y() > 0 && pixel.y() < window_height)
		{
			const int depth_index = (int)(pixel.y() * window_width + pixel.x());
			depth_buffer.at(depth_index) = p3d.z();
		}
	}

#if 0

	Eigen::Matrix4d proj = perspective_matrix(70.0, window_width / window_height, near_plane, far_plane);

	//std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> pointsProj;
	std::pair<std::vector<Eigen::Vector4d>, std::vector<Eigen::Vector4d>> pointsProj;
	for (auto p3d : points3D)
	{
		Eigen::Vector4d vertex0 = proj * T.first * p3d.homogeneous();
		Eigen::Vector4d vertex1 = proj * T.second * p3d.homogeneous();

		//vertex0 /= vertex0.w();
		//vertex1 /= vertex1.w();


		pointsProj.first.push_back(vertex0);
		pointsProj.second.push_back(vertex1);

		//Eigen::Vector3d pixel0 = vertex_to_window_coord(vertex0, fov_y, window_width / window_height, near_plane, far_plane, window_width, window_height);
		//Eigen::Vector3d pixel1 = vertex_to_window_coord(vertex1, fov_y, window_width / window_height, near_plane, far_plane, window_width, window_height);

		//pointsProj.first.push_back(pixel0);
		//pointsProj.second.push_back(pixel1);
	}

	//export_obj("../../data/proj_00.obj", pointsProj.first);
	//export_obj("../../data/proj_01.obj", pointsProj.second);
	//export_xyzd("../../data/proj_00.obj", pointsProj.first);
	//export_xyzd("../../data/proj_01.obj", pointsProj.second);
#endif

	//return 0;
	
	// Creating volume
	std::size_t slice_size = (voxel_count.x() + 1) * (voxel_count.y() + 1);
	std::vector<Voxeld> tsdf_volume((voxel_count.x() + 1) * (voxel_count.y() + 1) * (voxel_count.z() + 1));
	Eigen::Matrix4d volume_transformation = Eigen::Matrix4d::Identity();
	volume_transformation.col(3) << -(volume_size.x() / 2.0), -(volume_size.y() / 2.0), -(volume_size.z() / 2.0), 1.0;	// set translate

	int i = 0;
	for (int z = 0; z <= volume_size.z(); z += voxel_size.z())
	{
		for (int y = 0; y <= volume_size.y(); y += voxel_size.y())
		{
			for (int x = 0; x <= volume_size.x(); x += voxel_size.x(), i++)
			{
				tsdf_volume[i].point = Eigen::Vector3d(x, y, z);
				tsdf_volume[i].weight = i;
			}
		}
	}
	
	const Eigen::Matrix4d proj = perspective_matrix(fov_y, window_width / window_height, near_plane, far_plane);

	double max_sdf = -9999999;
	double min_sdf = FLT_MAX;
	for (auto it_volume = tsdf_volume.begin(); it_volume != tsdf_volume.end(); it_volume += slice_size)
	{
		auto z_slice_begin = it_volume;
		auto z_slice_end = it_volume + slice_size - 1;

		for (auto it = z_slice_begin; it != z_slice_end; ++it)
		{
			Eigen::Vector4d vg = volume_transformation * it->point.homogeneous();
			//Eigen::Vector4d v = T.second.inverse() * vg;
			Eigen::Vector4d v = T.first.inverse() * vg;
			v /= v.w();
			Eigen::Vector3d pixel = vertex_to_window_coord(v, fov_y, window_width / window_height, near_plane, far_plane, (int)window_width, (int)window_height);
						
			const Eigen::Vector4d p_clip = proj * v;
			const Eigen::Vector3d p_ndc = (p_clip / p_clip.w()).head<3>();

			Eigen::Vector3d p_window;
			p_window.x() = window_width / 2.0 * p_ndc.x() + window_width / 2.0;
			p_window.y() = window_height / 2.0 * p_ndc.y() + window_height / 2.0;
			p_window.z() = (far_plane - near_plane) / 2.0 * p_ndc.z() + (far_plane + near_plane) / 2.0;

#if 1		// kinect fusion
			// if v in camera frustum
			if (pixel.x() > 0 && pixel.x() < window_width && pixel.y() > 0 && pixel.y() < window_height)
			{
				//Eigen::Vector4d ti = T.second.inverse().col(3);
				Eigen::Vector4d ti = T.first.inverse().col(3);

				const int pixel_index = (int)(pixel.y() * window_width + pixel.x());
				const double Dp = depth_buffer.at(pixel_index);

				double sdf = (ti - vg).norm() - Dp;
				double tsdf = sdf;

				//if (sdf > 0)
				//	tsdf = std::fmin(1.0, sdf / MaxTruncation);
				//else
				//	tsdf = std::fmax(-1.0, sdf / MinTruncation);

				const double w = std::fmin(MaxWeight, it->weight + 1.0);
				const double tsdf_avg = (it->tsdf * it->weight + tsdf * w) / it->weight * w;

				it->tsdf = tsdf;
				it->weight = w;

				if (max_sdf < it->tsdf)
					max_sdf = it->tsdf;

				if (min_sdf > it->tsdf)
					min_sdf = it->tsdf;
			}
#else		// open fusion

			for (Eigen::Vector3d p3d : points3D)
			{
				Eigen::Vector4d ti = T.first.inverse().col(3);
				double sdf = (p3d.homogeneous() - ti).norm() - (vg - ti).norm();

				it->tsdf = sdf;
				it->weight = 0;

				if (max_sdf < it->tsdf)
					max_sdf = it->tsdf;

				if (min_sdf > it->tsdf)
					min_sdf = it->tsdf;
			}

#endif
			
		}
	}

	std::cout << std::fixed << "min max sdf " << min_sdf << ", " << max_sdf << std::endl;

	export_volume("../../data/volume.obj", tsdf_volume, volume_transformation);

	return EXIT_SUCCESS;
}


