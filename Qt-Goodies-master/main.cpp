#include "qDoubleRangeSlider.h"
#include <QtCore/QDebug>
#include <QtGui/QApplication>
#include <QtGui/QBoxLayout>
#include <QtGui/QCleanlooksStyle>
#include <QtGui/QMainWindow>
#include <QtGui/QMotifStyle>
#include <QtGui/QPlastiqueStyle>
#include <QtGui/QProxyStyle>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <cmath>

int main(int argc, char* argv[])
{
  QApplication app(argc, argv);
  app.setOrganizationName("Numerical Rocks");
  app.setApplicationName("Application Example");
  QStyle* style;
  style = new QCleanlooksStyle();
  app.setStyle(style);

  QMainWindow mainWin;
  mainWin.setContentsMargins(0, 0, 0, 0);

  QWidget* w = new QWidget(&mainWin);
  mainWin.setCentralWidget(w);

  QLayout* layout = new QVBoxLayout(w);
  w->setLayout(layout);

  QDoubleRangeSlider* slider;
  slider = new QDoubleRangeSlider(Qt::Horizontal, w);
  slider->setSizePolicy(QSizePolicy(QSizePolicy::Expanding,
                                    QSizePolicy::Expanding));
  slider->setCutoffRange(QPair<double, double>(1, 1000));
  slider->setLogarithmic(true);
  slider->setTickInterval(10);
  slider->setRange(qMakePair(0.0, 5.0));
  layout->addWidget(slider);

  /*
  QRangeSlider* rSlider;
  rSlider = new QRangeSlider(Qt::Horizontal, w);
  rSlider->setSizePolicy(QSizePolicy(QSizePolicy::Expanding,
                                    QSizePolicy::Expanding));
  rSlider->setCutoffRange(QPair<int, int>(1, 1024));
  rSlider->setLogarithmic(true);
  rSlider->setTickInterval(std::log(2.0));
  rSlider->setRange(qMakePair(0, 32));
  layout->addWidget(rSlider);
  */

  QSlider* slider2 = new QSlider(Qt::Horizontal);
  //slider2->setRange(0,0);
  layout->addWidget(slider2);

  mainWin.show();
  return app.exec();

}
