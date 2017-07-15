
#include "GLPointCloud.h"
#include <iostream>

#include "CudaKernels/CudaKernels.h"

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>


static struct cudaGraphicsResource *cuda_vb_resource;
static int accum = 0;
static bool direction = true;


GLPointCloud::GLPointCloud() : GLModel()
{
}


GLPointCloud::~GLPointCloud()
{
	cleanupGL();
}


GLuint GLPointCloud::vertexBufferId() const
{
	return vertexBuf.buffer.bufferId();
}


void GLPointCloud::initGL()
{
	initializeOpenGLFunctions();

	vertexBuf.buffer.create();
	colorBuf.buffer.create();
}


void GLPointCloud::cleanupGL()
{
	vertexBuf.buffer.destroy();
	colorBuf.buffer.destroy();
}


void GLPointCloud::setVertices(const float* vertices, uint count, uint tuple_size)
{
	vertexBuf.location = 0;
	vertexBuf.count = count;
	vertexBuf.tuple = tuple_size;
	vertexBuf.stride = sizeof(float) * tuple_size;


	vertexBuf.buffer.bind();
	vertexBuf.buffer.allocate(vertices, static_cast<int>(vertexBuf.count * vertexBuf.stride));
	
	cudaGraphicsGLRegisterBuffer(&cuda_vb_resource, vertexBuf.buffer.bufferId(), cudaGraphicsMapFlagsWriteDiscard);
	vertexBuf.buffer.release();
}



void GLPointCloud::setColors(const float* colors, uint count, uint tuple_size)
{
	colorBuf.location = 1;
	colorBuf.count = count;
	colorBuf.tuple = tuple_size;
	colorBuf.stride = sizeof(float) * tuple_size;


	colorBuf.buffer.bind();
	colorBuf.buffer.allocate(colors, static_cast<int>(colorBuf.count * colorBuf.stride));
	colorBuf.buffer.release();
}




void GLPointCloud::render(QOpenGLShaderProgram *program)
{
	if (!vertexBuf.buffer.isCreated())
		return;

	//////////////////////////////////////////
	//
	// begin Cuda code
	//
	float* d_verts_ptr;
	size_t num_bytes;
	cudaGraphicsMapResources(1, &cuda_vb_resource, 0);
	cudaGraphicsResourceGetMappedPointer((void **)&d_verts_ptr, &num_bytes, cuda_vb_resource);
	float dir = direction ? 1.01f : 0.99f;
		
	if (++accum % 10 == 0)
		direction = !direction;

	cuda_kernel(d_verts_ptr, vertexBuf.count, dir);
	cudaGraphicsUnmapResources(1, &cuda_vb_resource, 0);
	//
	// end Cuda code
	//
	//////////////////////////////////////////



	program->bind();

	vertexBuf.buffer.bind();
	program->setAttributeBuffer(vertexBuf.location, GL_FLOAT, 0, vertexBuf.tuple, vertexBuf.stride);
	program->enableAttributeArray(vertexBuf.location);

	colorBuf.buffer.bind();
	program->setAttributeBuffer(colorBuf.location, GL_FLOAT, 0, colorBuf.tuple, colorBuf.stride);
	program->enableAttributeArray(colorBuf.location);

    // Draw geometry 
	glDrawArrays(GL_POINTS, 0, vertexBuf.count);

	vertexBuf.buffer.release();
	colorBuf.buffer.release();
	program->release();
}

