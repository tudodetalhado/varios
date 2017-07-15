// Copyright (C) 2012-2016 The VPaint Developers.
// See the COPYRIGHT file at the top-level directory of this distribution
// and at https://github.com/dalboris/vpaint/blob/master/COPYRIGHT
//
// This file is part of VPaint, a vector graphics editor. It is subject to the
// license terms and conditions in the LICENSE.MIT file found in the top-level
// directory of this distribution and at http://opensource.org/licenses/MIT

#include "SpinBox.h"

#include <QVBoxLayout>

SpinBox::SpinBox(QWidget *parent) :
    QWidget(parent),
    caption_(0),
    spinBox_(0)
{
    // Caption
    caption_ = new QLabel();
    QFont labelFont = font();
    labelFont.setPixelSize(11);
    caption_->setFont(labelFont);
    caption_->setAlignment(Qt::AlignCenter);

    // Spin box
    spinBox_ = new QDoubleSpinBox();
    spinBox_->setRange(0.0, 999.99);

    // Layout
    QVBoxLayout * layout = new QVBoxLayout();
    layout->addWidget(caption_);
    layout->addWidget(spinBox_);
    layout->setSpacing(0);
    layout->setContentsMargins(0,0,0,0);
    setLayout(layout);
    setFixedHeight(40);

    // Signal forwarding
    connect(spinBox_, SIGNAL(valueChanged(double)), this, SIGNAL(valueChanged(double)));
}

QString SpinBox::caption() const
{
    return caption_->text();
}

void SpinBox::setCaption(const QString & caption)
{
    caption_->setText(caption);
}

double SpinBox::value() const
{
    return spinBox_->value();
}

void SpinBox::setValue(double val)
{
    spinBox_->setValue(val);
}
