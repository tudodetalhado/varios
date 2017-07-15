#include <QApplication>
#include <QMainWindow>
#include "pclKinect.h"

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  KinectViewer w;
  w.show ();

  return a.exec ();
}
