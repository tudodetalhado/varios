#ifndef __GL_WIDGET_H__
#define __GL_WIDGET_H__

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>
#include <QMatrix4x4>
#include <QSharedPointer>
#include <QOpenGLShaderProgram>

#include "PCLMesh.h"
#include "trackball.hpp"

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT

public:
	GLWidget(QWidget *parent = 0);
	~GLWidget();

	QSize minimumSizeHint() const Q_DECL_OVERRIDE{ return QSize(50, 50); };
	QSize sizeHint() const Q_DECL_OVERRIDE{ return QSize(1280, 400); };

	

public slots:
	void cleanup();
	void loadMesh(QString fileName);

protected:
	void initializeGL() Q_DECL_OVERRIDE;
	void paintGL() Q_DECL_OVERRIDE;
	void resizeGL(int width, int height) Q_DECL_OVERRIDE;
	void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void mouseReleaseEvent(QMouseEvent * event) Q_DECL_OVERRIDE;
	void wheelEvent(QWheelEvent * event) Q_DECL_OVERRIDE;

	void initShaders();
	void initTextures();

private:
	QOpenGLVertexArrayObject m_vao;
	QOpenGLBuffer m_vbo;
	QOpenGLShaderProgram program;
	QOpenGLTexture *texture;
	QMatrix4x4 projection;
	QMatrix4x4 modelview;
	PCLMesh mesh;

	Tucano::Trackball camera;
};


#endif // __GL_WIDGET_H__
