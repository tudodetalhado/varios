
#ifndef _GL_BASE_WIDGET_H_
#define _GL_BASE_WIDGET_H_

#include "QOpenGLTrackballWidget.h"
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

class GLModel;


class GLBaseWidget : public QOpenGLTrackballWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit GLBaseWidget(QWidget *parent = 0);
    ~GLBaseWidget();

	void setModel(GLModel* gl_model);

protected:

    void initializeGL() Q_DECL_OVERRIDE;
    void resizeGL(int w, int h) Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;

    void initShaders();

private:
    QOpenGLShaderProgram program;
    GLModel* model;

    QMatrix4x4 projection;
};

#endif // _GL_BASE_WIDGET_H_
