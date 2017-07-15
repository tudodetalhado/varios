#include "DepthBasics.h"
#include <QApplication>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CDepthBasics w;
    w.show();

    return a.exec();
}
