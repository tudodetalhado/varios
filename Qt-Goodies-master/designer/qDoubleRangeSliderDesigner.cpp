#include "qDoubleRangeSliderDesigner.h"

#include "../qDoubleRangeSlider.h"

#include <QtCore/QtPlugin>
#ifndef QTGOODIES_INCLUDE_PREFIX
#define QTGOODIES_INCLUDE_PREFIX
#endif


QDoubleSliderDesigner::QDoubleSliderDesigner(QObject *parent)
  : initialized_(false)
{

}

void QDoubleSliderDesigner::initialize(QDesignerFormEditorInterface *)
{
  if (initialized_)
    return;

  initialized_ = true;
}

bool QDoubleSliderDesigner::isInitialized() const
{
  return initialized_;
}

QWidget *QDoubleSliderDesigner::createWidget(QWidget *parent)
{
  return new QDoubleRangeSlider(parent);
}

QString QDoubleSliderDesigner::name() const
{
  return "QDoubleSlider";
}

QString QDoubleSliderDesigner::group() const
{
  return "Input Widgets";
}

QIcon QDoubleSliderDesigner::icon() const
{
  return QIcon();
}

QString QDoubleSliderDesigner::toolTip() const
{
  return "";
}

QString QDoubleSliderDesigner::whatsThis() const
{
  return "";
}

bool QDoubleSliderDesigner::isContainer() const
{
  return false;
}

QString QDoubleSliderDesigner::domXml() const
{
  return "<ui language=\"c++\">\n"
    " <widget class=\"QDoubleSlider\" name=\"QDoubleSlider\">\n"
    " </widget>\n"
    "</ui>\n";
}

QString QDoubleSliderDesigner::includeFile() const
{
  return QTGOODIES_INCLUDE_PREFIX  "qDoubleRangeSlider.h";
}

Q_EXPORT_PLUGIN2(QDoubleSliderDesigner, QDoubleSliderDesigner)
