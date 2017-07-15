// Copyright (C) 2012-2016 The VPaint Developers.
// See the COPYRIGHT file at the top-level directory of this distribution
// and at https://github.com/dalboris/vpaint/blob/master/COPYRIGHT
//
// This file is part of VPaint, a vector graphics editor. It is subject to the
// license terms and conditions in the LICENSE.MIT file found in the top-level
// directory of this distribution and at http://opensource.org/licenses/MIT

#ifndef XMLSTREAMCONVERTER_H
#define XMLSTREAMCONVERTER_H

#include "XmlStreamTraverser.h"

class XmlStreamReader;
class XmlStreamWriter;

class XmlStreamConverter: public XmlStreamTraverser
{
public:
    XmlStreamConverter(XmlStreamReader & in, XmlStreamWriter & out);
    virtual ~XmlStreamConverter() {}

    XmlStreamReader & in();
    XmlStreamWriter & out();

private:
    XmlStreamWriter & out_;
};

#endif // XMLSTREAMCONVERTER_H
