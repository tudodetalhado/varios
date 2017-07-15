#include "PCLMesh.h"

#include <QVector2D>
#include <QVector3D>

struct VertexData
{
	QVector3D position;
	QVector2D texCoord;
};


PCLMesh::PCLMesh(): indexBuf(QOpenGLBuffer::IndexBuffer)
{
}

PCLMesh::~PCLMesh()
{
}


bool PCLMesh::load(const std::string& meshFilePathName)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(meshFilePathName, cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file: ", meshFilePathName);
		return false;
	}
	else
	{
		bindVertexArray();
		return true;
	}
	
}


void PCLMesh::cleanup()
{
	arrayBuf.destroy();
	indexBuf.destroy();
}

void PCLMesh::initializeGL()
{
	initializeOpenGLFunctions();

	glPointSize(3);

	// Generate 2 VBOs
	arrayBuf.create();
	//indexBuf.create();

	//bindVertexArray();
}


void PCLMesh::bindVertexArray()
{
	// For cube we would need only 8 vertices but we have to
	// duplicate vertex for each face because texture coordinate
	// is different.
	VertexData vertices[] = {
		// Vertex data for face 0
		{ QVector3D(-1.0f, -1.0f, 1.0f), QVector2D(0.0f, 0.0f) },  // v0
		{ QVector3D(1.0f, -1.0f, 1.0f), QVector2D(0.33f, 0.0f) }, // v1
		{ QVector3D(-1.0f, 1.0f, 1.0f), QVector2D(0.0f, 0.5f) },  // v2
		{ QVector3D(1.0f, 1.0f, 1.0f), QVector2D(0.33f, 0.5f) }, // v3

		// Vertex data for face 1
		{ QVector3D(1.0f, -1.0f, 1.0f), QVector2D(0.0f, 0.5f) }, // v4
		{ QVector3D(1.0f, -1.0f, -1.0f), QVector2D(0.33f, 0.5f) }, // v5
		{ QVector3D(1.0f, 1.0f, 1.0f), QVector2D(0.0f, 1.0f) },  // v6
		{ QVector3D(1.0f, 1.0f, -1.0f), QVector2D(0.33f, 1.0f) }, // v7

		// Vertex data for face 2
		{ QVector3D(1.0f, -1.0f, -1.0f), QVector2D(0.66f, 0.5f) }, // v8
		{ QVector3D(-1.0f, -1.0f, -1.0f), QVector2D(1.0f, 0.5f) },  // v9
		{ QVector3D(1.0f, 1.0f, -1.0f), QVector2D(0.66f, 1.0f) }, // v10
		{ QVector3D(-1.0f, 1.0f, -1.0f), QVector2D(1.0f, 1.0f) },  // v11

		// Vertex data for face 3
		{ QVector3D(-1.0f, -1.0f, -1.0f), QVector2D(0.66f, 0.0f) }, // v12
		{ QVector3D(-1.0f, -1.0f, 1.0f), QVector2D(1.0f, 0.0f) },  // v13
		{ QVector3D(-1.0f, 1.0f, -1.0f), QVector2D(0.66f, 0.5f) }, // v14
		{ QVector3D(-1.0f, 1.0f, 1.0f), QVector2D(1.0f, 0.5f) },  // v15

		// Vertex data for face 4
		{ QVector3D(-1.0f, -1.0f, -1.0f), QVector2D(0.33f, 0.0f) }, // v16
		{ QVector3D(1.0f, -1.0f, -1.0f), QVector2D(0.66f, 0.0f) }, // v17
		{ QVector3D(-1.0f, -1.0f, 1.0f), QVector2D(0.33f, 0.5f) }, // v18
		{ QVector3D(1.0f, -1.0f, 1.0f), QVector2D(0.66f, 0.5f) }, // v19

		// Vertex data for face 5
		{ QVector3D(-1.0f, 1.0f, 1.0f), QVector2D(0.33f, 0.5f) }, // v20
		{ QVector3D(1.0f, 1.0f, 1.0f), QVector2D(0.66f, 0.5f) }, // v21
		{ QVector3D(-1.0f, 1.0f, -1.0f), QVector2D(0.33f, 1.0f) }, // v22
		{ QVector3D(1.0f, 1.0f, -1.0f), QVector2D(0.66f, 1.0f) }  // v23
	};

	// Indices for drawing cube faces using triangle strips.
	// Triangle strips can be connected by duplicating indices
	// between the strips. If connecting strips have opposite
	// vertex order then last index of the first strip and first
	// index of the second strip needs to be duplicated. If
	// connecting strips have same vertex order then only last
	// index of the first strip needs to be duplicated.
	GLushort indices[] = {
		0, 1, 2, 3, 3,     // Face 0 - triangle strip ( v0,  v1,  v2,  v3)
		4, 4, 5, 6, 7, 7, // Face 1 - triangle strip ( v4,  v5,  v6,  v7)
		8, 8, 9, 10, 11, 11, // Face 2 - triangle strip ( v8,  v9, v10, v11)
		12, 12, 13, 14, 15, 15, // Face 3 - triangle strip (v12, v13, v14, v15)
		16, 16, 17, 18, 19, 19, // Face 4 - triangle strip (v16, v17, v18, v19)
		20, 20, 21, 22, 23      // Face 5 - triangle strip (v20, v21, v22, v23)
	};

	
	// Transfer vertex data to VBO 0
	arrayBuf.bind();
	//arrayBuf.allocate(vertices, 24 * sizeof(VertexData));
	arrayBuf.allocate(reinterpret_cast<void *>(&cloud.points[0]), cloud.size() * sizeof(pcl::PointXYZRGB));

	qDebug() << cloud.size();

	for (auto it = cloud.points.begin(); it != cloud.points.end(); ++it)
	{
		qDebug() << it->x << ", " << it->y << ", " << it->z;
	}
	// Transfer index data to VBO 1
	//indexBuf.bind();
	//indexBuf.allocate(indices, 34 * sizeof(GLushort));
	

	
}



void PCLMesh::draw(QOpenGLShaderProgram *program)
{
	// Tell OpenGL which VBOs to use
	arrayBuf.bind();
	//indexBuf.bind();

	// Offset for position
	quintptr offset = 0;

	// Tell OpenGL programmable pipeline how to locate vertex position data
	int vertexLocation = program->attributeLocation("a_position");
	program->enableAttributeArray(vertexLocation);
	program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(pcl::PointXYZRGB));

	// Offset for texture coordinate
	offset += sizeof(QVector3D);

	// Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
	/*int texcoordLocation = program->attributeLocation("a_texcoord");
	program->enableAttributeArray(texcoordLocation);
	program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));*/

	// Draw cube geometry using indices from VBO 1
	//glDrawElements(GL_POINTS, 34, GL_UNSIGNED_SHORT, 0);
	glDrawArrays(GL_POINTS, 0, cloud.size());
}

