#pragma once
#include "LabelViewer.h"
#include <QWidget>
#include <QMouseEvent>

LabelViewer::LabelViewer(QWidget *parent)
    : QLabel(parent)
{
}

void LabelViewer::mouseDoubleClickEvent(QMouseEvent *e)
{
    emit cliqueDuplo();
}
