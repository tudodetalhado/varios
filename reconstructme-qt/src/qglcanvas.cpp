/** @file
  * @copyright Copyright (c) 2013 PROFACTOR GmbH. All rights reserved. 
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are
  * met:
  *
  *     * Redistributions of source code must retain the above copyright
  * notice, this list of conditions and the following disclaimer.
  *     * Redistributions in binary form must reproduce the above
  * copyright notice, this list of conditions and the following disclaimer
  * in the documentation and/or other materials provided with the
  * distribution.
  *     * Neither the name of Profactor GmbH nor the names of its
  * contributors may be used to endorse or promote products derived from
  * this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * @authors christoph.kopf@profactor.at
  *          florian.eckerstorfer@profactor.at
  */

#ifndef QGLCANVAS_H
#define QGLCANVAS_H

#pragma once

#include "qglcanvas.h"

#include <QSize>

#include <iostream>

namespace ReconstructMeGUI {

  QGLCanvas::QGLCanvas(QWidget* parent) : QGLWidget(parent),
    _width(3),
    _height(3)
  {
    _img = std::shared_ptr<QImage>(new QImage(_width, _height, QImage::Format_RGB888));
    fill();
  }

  void QGLCanvas::fill(const QColor &color) {
    _img->fill(color);
  }

  void QGLCanvas::set_image(int width, int height, const void *data, int length) {
    
    if (width < 0 || height < 0 || data == 0)
      return;

    if (_width != width || _height != height) {
      _width = width;
      _height = height;
      _img = std::shared_ptr<QImage>(new QImage(_width, _height, QImage::Format_RGB888));
    }

    memcpy((void*)_img->bits(), data, length); 
    repaint();

  }

  void QGLCanvas::paintEvent(QPaintEvent* ev) {
    QPainter p(this);

    //Set the painter to use a smooth scaling algorithm.
    p.setRenderHint(QPainter::SmoothPixmapTransform, 1);
    p.drawImage(this->rect(), *_img.get());
    
    p.end();
  }

}

#endif