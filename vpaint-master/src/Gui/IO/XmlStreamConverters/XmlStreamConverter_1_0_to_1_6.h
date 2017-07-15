// Copyright (C) 2012-2016 The VPaint Developers.
// See the COPYRIGHT file at the top-level directory of this distribution
// and at https://github.com/dalboris/vpaint/blob/master/COPYRIGHT
//
// This file is part of VPaint, a vector graphics editor. It is subject to the
// license terms and conditions in the LICENSE.MIT file found in the top-level
// directory of this distribution and at http://opensource.org/licenses/MIT

#ifndef XMLSTREAMCONVERTER_1_0_TO_1_6_H
#define XMLSTREAMCONVERTER_1_0_TO_1_6_H

#include "IO/XmlStreamConverter.h"

class XmlStreamConverter_1_0_to_1_6: public XmlStreamConverter
{
public:
    XmlStreamConverter_1_0_to_1_6(XmlStreamReader & in, XmlStreamWriter & out);

    void begin();
    void end();
    void pre();
    void post();
};

#endif // XMLSTREAMCONVERTER_1_0_TO_1_6_H
