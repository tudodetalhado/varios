
#include <QApplication>
#include <QKeyEvent>
#include <iostream>
#include "GLRaycastTextureWidget.h"


int main(int argc, char **argv)
{
	QApplication app(argc, argv);

	GLRaycastTextureWidget glwidget;
	glwidget.resize(512, 512);
	glwidget.setWindowTitle("GL Widget");
	glwidget.show();

	return app.exec();
}
