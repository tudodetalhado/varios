
#include "GLRaycastTextureWidget.h"
#include "GLModel.h"
#include "GLQuad.h"
#include <QMouseEvent>
#include <QTimer>
#include <math.h>
#include "CudaKernels/CudaKernels.h"

// CUDA Runtime, Interop, and includes
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <vector_types.h>
#include <vector_functions.h>
#include <driver_functions.h>

// CUDA utilities
#include <helper_cuda.h>
#include <helper_cuda_gl.h>

// Helper functions
#include <helper_cuda.h>
#include <helper_functions.h>
#include <helper_timer.h>

struct cudaGraphicsResource *cuda_pixel_buffer; // CUDA Graphics Resource (to transfer PBO)
StopWatchInterface *cuda_timer = 0;

static ushort g_width = 504;
static ushort g_height = 504;

GLRaycastTextureWidget::GLRaycastTextureWidget(QWidget *parent) :
	QOpenGLTrackballWidget(parent)
	, pixelBuf(nullptr)
	, texture(nullptr)
	, cudaInitialized(false)
{
	QTimer* updateTimer = new QTimer(this);
	connect(updateTimer, SIGNAL(timeout()), this, SLOT(update()));
	updateTimer->start(33);

	weelSpeed = 0.001f;
	distance = -4;
}


GLRaycastTextureWidget::~GLRaycastTextureWidget()
{
	// cudaDeviceReset causes the driver to clean up all state. While
	// not mandatory in normal operation, it is good practice.  It is also
	// needed to ensure correct operation when the application is being
	// profiled. Calling cudaDeviceReset causes all profile data to be
	// flushed before the application exits
	cudaDeviceReset();
}



void GLRaycastTextureWidget::initializeGL()
{
    initializeOpenGLFunctions();

	std::stringstream info;
	info << "OpenGl information: VENDOR:       " << (const char*)glGetString(GL_VENDOR) << std::endl
		<< "                    RENDERER:     " << (const char*)glGetString(GL_RENDERER) << std::endl
		<< "                    VERSION:      " << (const char*)glGetString(GL_VERSION) << std::endl
		<< "                    GLSL VERSION: " << (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
	std::cout << std::endl << info.str() << std::endl << std::endl;

    glClearColor(0, 0, 0, 1);

    initShaders();

    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);

    // Enable back face culling
    glEnable(GL_CULL_FACE);

	quad = new GLQuad();
	quad->initGL();

	cudaInit();

	initPixelBuffer();

}


void GLRaycastTextureWidget::initShaders()
{
    // Compile vertex shader
	if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/vertices.vert"))
        close();

    // Compile fragment shader
	if (!program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/vertices.frag"))
        close();

    // Link shader pipeline
    if (!program.link())
        close();

    // Bind shader pipeline for use
    if (!program.bind())
        close();
}





void GLRaycastTextureWidget::resizeGL(int w, int h)
{
	const float fovy = 45.0f;
	const float aspect_ratio = (float)w / (float) h;
	const float near_plane = 0.1f;
	const float far_plane = 10240.0f;

	glViewport(0, 0, w, h);

    // Reset projection
    projection.setToIdentity();

    // Set perspective projection
	projection.perspective(fovy, aspect_ratio, near_plane, far_plane);
}



void GLRaycastTextureWidget::paintGL()
{
	if (!texture)
		return;

	sdkStartTimer(&cuda_timer);

	cudaRender();

	// display results
	glClear(GL_COLOR_BUFFER_BIT);

	// draw image from PBO
	glDisable(GL_DEPTH_TEST);

	texture->bind();

	// copy from pbo to texture
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	pixelBuf->bind();
	texture->setData(QOpenGLTexture::PixelFormat::RGBA, QOpenGLTexture::PixelType::UInt8, (void*)nullptr);
	pixelBuf->release();


	// Clear color and depth buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	quad->render();


	texture->release();

	sdkStopTimer(&cuda_timer);
}





void GLRaycastTextureWidget::cudaRender()
{
	if (!pixelBuf || !pixelBuf->isCreated())
		return;

	// map PBO to get CUDA device pointer
	uint *image_dev_ptr;
	// map PBO to get CUDA device pointer
	checkCudaErrors(cudaGraphicsMapResources(1, &cuda_pixel_buffer, 0));
	size_t num_bytes;
	checkCudaErrors(cudaGraphicsResourceGetMappedPointer((void **)&image_dev_ptr, &num_bytes, cuda_pixel_buffer));
	
	// clear image
	//checkCudaErrors(cudaMemset(image_dev_ptr, 0, g_width * g_height * 4));

	QMatrix4x4 view_matrix;
	view_matrix.translate(0, 0, distance);
	view_matrix.rotate(rotation);

	QMatrix4x4 view_matrix_inv = view_matrix.transposed().inverted();

	// call cuda kernel
	raycast_box(
		image_dev_ptr, 
		g_width, 
		g_height, 
		view_matrix_inv.data(),
		make_ushort3(1, 1, 1));

	checkCudaErrors(cudaDeviceSynchronize());
	getLastCudaError("kernel failed");

	checkCudaErrors(cudaGraphicsUnmapResources(1, &cuda_pixel_buffer, 0));
}


void GLRaycastTextureWidget::cudaInit()
{
	cudaInitialized = false;
	int devID = gpuGetMaxGflopsDeviceId();
	if (devID < 0)
		std::cerr << "Error: No CUDA capable devices found" << std::endl;
	else
		checkCudaErrors(cudaGLSetGLDevice(devID));

	cudaInitialized = true;

	sdkCreateTimer(&cuda_timer);
}



void GLRaycastTextureWidget::initPixelBuffer()
{
	if (pixelBuf)
	{
		// unregister this buffer object from CUDA C
		checkCudaErrors(cudaGraphicsUnregisterResource(cuda_pixel_buffer));

		pixelBuf->destroy();
		texture->destroy();
		delete pixelBuf;
		delete texture;
	}

	std::vector<GLubyte> image;
	for (GLuint i = 0; i < g_width; i++)
	{
		for (GLuint j = 0; j < g_height; j++)
		{
			GLuint c = ((((i & 0x4) == 0) ^ ((j & 0x4)) == 0)) * 255;
			image.push_back((GLubyte)c);
			image.push_back((GLubyte)c);
			image.push_back((GLubyte)c);
			image.push_back(255);
		}
	}

	pixelBuf = new QOpenGLBuffer(QOpenGLBuffer::PixelUnpackBuffer);
	pixelBuf->setUsagePattern(QOpenGLBuffer::StreamDraw);
	pixelBuf->create();
	pixelBuf->bind();
	pixelBuf->allocate(image.data(), g_width * g_height * sizeof(GLubyte) * 4);
	pixelBuf->release();

	// register this buffer object with CUDA
	checkCudaErrors(cudaGraphicsGLRegisterBuffer(&cuda_pixel_buffer, pixelBuf->bufferId(), cudaGraphicsMapFlagsWriteDiscard));

	texture = new QOpenGLTexture(QOpenGLTexture::Target2D);
	texture->create();
	texture->bind();
	texture->setMinificationFilter(QOpenGLTexture::Nearest);
	texture->setMagnificationFilter(QOpenGLTexture::Nearest);
	texture->setWrapMode(QOpenGLTexture::ClampToEdge);
	texture->setSize(g_width, g_height);
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	texture->setFormat(QOpenGLTexture::TextureFormat::RGBA8_UNorm);
	texture->allocateStorage(QOpenGLTexture::PixelFormat::RGBA, QOpenGLTexture::PixelType::UInt8);
	//texture->setData(QOpenGLTexture::PixelFormat::RGBA, QOpenGLTexture::PixelType::UInt8, image.data());
	texture->release();

}



