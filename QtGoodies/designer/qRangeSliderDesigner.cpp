#include "qRangeSliderDesigner.h"

#include "../qRangeSlider.h"

#include <QtCore/QtPlugin>
#ifndef QTGOODIES_INCLUDE_PREFIX
#define QTGOODIES_INCLUDE_PREFIX
#endif


QRangeSliderDesigner::QRangeSliderDesigner(QObject *parent)
  : initialized_(false)
{

}

void QRangeSliderDesigner::initialize(QDesignerFormEditorInterface *)
{
  if (initialized_)
    return;

  initialized_ = true;
}

bool QRangeSliderDesigner::isInitialized() const
{
  return initialized_;
}

QWidget *QRangeSliderDesigner::createWidget(QWidget *parent)
{
  return new QRangeSlider(parent);
}

QString QRangeSliderDesigner::name() const
{
  return "QRangeSlider";
}

QString QRangeSliderDesigner::group() const
{
  return "Input Widgets";
}

QIcon QRangeSliderDesigner::icon() const
{
  return QIcon();
}

QString QRangeSliderDesigner::toolTip() const
{
  return "";
}

QString QRangeSliderDesigner::whatsThis() const
{
  return "";
}

bool QRangeSliderDesigner::isContainer() const
{
  return false;
}

QString QRangeSliderDesigner::domXml() const
{
  return "<ui language=\"c++\">\n"
    " <widget class=\"QRangeSlider\" name=\"QRangeSlider\">\n"
    " </widget>\n"
    "</ui>\n";
}

QString QRangeSliderDesigner::includeFile() const
{
  return QTGOODIES_INCLUDE_PREFIX  "qRangeSlider.h";
}

Q_EXPORT_PLUGIN2(QRangeSliderDesigner, QRangeSliderDesigner)
