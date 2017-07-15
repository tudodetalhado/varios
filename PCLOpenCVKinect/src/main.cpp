#include "pclopencvKinect.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  KinectViewer w;
  w.show ();

  return a.exec ();
}
