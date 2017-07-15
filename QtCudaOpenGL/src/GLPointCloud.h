
#ifndef _GL_POINT_CLOUD_H_
#define _GL_POINT_CLOUD_H_

#include "GLModel.h"

struct GLBufferArray
{
	QOpenGLBuffer buffer;
	uint location;
	uint count;
	uint tuple;
	uint stride;
};

class GLPointCloud : public GLModel
{
public:
	GLPointCloud();
	virtual ~GLPointCloud();

	void render(QOpenGLShaderProgram *program);

	void setVertices(const float* vertices, uint count, uint tuple_size);
	void setColors(const float* colors, uint count, uint tuple_size);

	GLuint vertexBufferId() const;

public slots:

	void initGL();
	void cleanupGL();

private:
	GLBufferArray vertexBuf;
	GLBufferArray colorBuf;
};

#endif // _GL_POINT_CLOUD_H_
