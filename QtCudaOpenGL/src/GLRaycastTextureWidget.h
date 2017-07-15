
#ifndef _GL_RAYCAST_TEXTURE_WIDGET_H_
#define _GL_RAYCAST_TEXTURE_WIDGET_H_

#include "QOpenGLTrackballWidget.h"
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

class GLModel;
class GLQuad;


class GLRaycastTextureWidget : public QOpenGLTrackballWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit GLRaycastTextureWidget(QWidget *parent = 0);
    ~GLRaycastTextureWidget();

protected:

    void initializeGL() Q_DECL_OVERRIDE;
    void resizeGL(int w, int h) Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;

    void initShaders();
	void initPixelBuffer();

	void cudaInit();
	void cudaRender();

private:
    QOpenGLShaderProgram program;
	
	QOpenGLBuffer*	pixelBuf;
	QOpenGLTexture*  texture;
	GLQuad* quad;

    QMatrix4x4 projection;

	bool cudaInitialized;

};

#endif // _GL_RAYCAST_TEXTURE_WIDGET_H_
