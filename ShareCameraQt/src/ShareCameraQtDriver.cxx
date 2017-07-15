#include <QApplication>
#include "ShareCameraQt.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
 
int main( int argc, char** argv )
{
  // QT Stuff
  QApplication app( argc, argv );
 
  ShareCameraQt shareCameraQt;
  shareCameraQt.show();
 
  return app.exec();
}