#ifndef WINDOW_H
#define WINDOW_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QOpenGLWindow>
#include <QPaintDeviceWindow>
#include "transform3d.h"

class QOpenGLShaderProgram;

class Window : public QOpenGLWidget,
               protected QOpenGLFunctions
{
  Q_OBJECT

// OpenGL Events
public:
  Window();
  ~Window();
  void initializeGL();
  void resizeGL(int width, int height);
  void paintGL();
  void teardownGL();
protected slots:
  void update();

private:
  // OpenGL State Information
  QOpenGLBuffer m_vertex;
  QOpenGLVertexArrayObject m_object;
  QOpenGLShaderProgram *m_program;

  // Shader Information
  int u_modelToWorld;
  int u_worldToView;
  QMatrix4x4 m_projection;
  Transform3D m_transform;

  // Private Helpers
  void printVersionInformation();
};

#endif // WINDOW_H
