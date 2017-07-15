#ifndef _PCL_MESH_H_
#define _PCL_MESH_H_

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class PCLMesh : protected QOpenGLFunctions
{
public:
	PCLMesh();
	virtual ~PCLMesh();

	bool load(const std::string& meshFilePathName);

	void initializeGL();
	void cleanup();

	void draw(QOpenGLShaderProgram *program);

protected:

	void bindVertexArray();

private:

	QOpenGLBuffer arrayBuf;
	QOpenGLBuffer indexBuf;

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
};

#endif // _PCL_MESH_H_
