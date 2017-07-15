
#include "GLBaseWidget.h"
#include "GLModel.h"
#include <QMouseEvent>
#include <QTimer>
#include <math.h>

GLBaseWidget::GLBaseWidget(QWidget *parent) :
	QOpenGLTrackballWidget(parent),
	model(nullptr)
{
	QTimer* updateTimer = new QTimer(this);
	connect(updateTimer, SIGNAL(timeout()), this, SLOT(update()));
	updateTimer->start(33);

	distance = -150;
}


GLBaseWidget::~GLBaseWidget()
{
}


void GLBaseWidget::setModel(GLModel* gl_model)
{
	model = gl_model;
}


void GLBaseWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0, 0, 0, 1);

    initShaders();

    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);

    // Enable back face culling
    glEnable(GL_CULL_FACE);

	if (model != nullptr)
		model->initGL();
}


void GLBaseWidget::initShaders()
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





void GLBaseWidget::resizeGL(int w, int h)
{
	const float fovy = 40.0f;
	const float aspect_ratio = 1.77f;
	const float near_plane = 0.1f;
	const float far_plane = 10240.0f;

    // Reset projection
    projection.setToIdentity();

    // Set perspective projection
	projection.perspective(fovy, aspect_ratio, near_plane, far_plane);
}



void GLBaseWidget::paintGL()
{
    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Calculate model view transformation
    QMatrix4x4 view_matrix;
	view_matrix.translate(0, 0, distance);
	view_matrix.rotate(rotation);
	
	QMatrix4x4 model_matrix;
	//model_matrix.rotate(180, 0, 1, 0);

    // Set modelview-projection matrix
	program.setUniformValue("mvp_matrix", projection * view_matrix * model_matrix);

    // Draw cube geometry
	if (model != nullptr)
		model->render(&program);
}
