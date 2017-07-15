#include <QApplication>
#include <QMainWindow>
#include "window.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);

  // Set OpenGL Version information
  // Note: This format must be set before show() is called.
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setVersion(3, 3);

  // Set widget up
  Window *widget = new Window;
  widget->setFormat(format);

  // Set the window up
  QMainWindow window;
  window.setCentralWidget(widget);
  window.resize(QSize(800, 600));
  window.show();

  return app.exec();
}
