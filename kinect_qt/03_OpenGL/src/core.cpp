#include <QtGui/QApplication>
#include <dialog.h>

int main(int argc, char** argv)
{
	QApplication app(argc, argv);

	Kinect::Dialog dlg;
	dlg.show();

 	return(app.exec());
}
