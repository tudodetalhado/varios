// Copyright (C) 2012-2016 The VPaint Developers.
// See the COPYRIGHT file at the top-level directory of this distribution
// and at https://github.com/dalboris/vpaint/blob/master/COPYRIGHT
//
// This file is part of VPaint, a vector graphics editor. It is subject to the
// license terms and conditions in the LICENSE.MIT file found in the top-level
// directory of this distribution and at http://opensource.org/licenses/MIT

#ifndef CSSCOLOR_H
#define CSSCOLOR_H

#include "Color.h"

#include <QString>

class CssColor
{
public:
    // Constructors
    CssColor(int r=0, int g=0, int b=0, double a=1.0); // expects RGB in [0,255] and A in [0,1]
    CssColor(const QString & c);                       // expects string of the form "rgba(r,b,b,a)", same ranges as above
    CssColor(const double * c);                        // expects an array of size 4 with RGBA values all in [0,1]

    // Get
    int r() const;
    int g() const;
    int b() const;
    double a() const;

    double rF() const;
    double gF() const;
    double bF() const;
    double aF() const;

    Color toColor() const;

    // Set
    void setRgba(int r, int g, int b, double a);
    void setR(int r);
    void setG(int g);
    void setB(int b);
    void setA(double a);

    void setRgbaF(double r, double g, double b, double a);
    void setRF(double r);
    void setGF(double g);
    void setBF(double b);
    void setAF(double a);

    // String input/output as "rgba(r,g,b,a)"
    QString toString() const;
    void fromString(const QString & c);

private:
    int r_, g_, b_; // [0  , 255]
    double a_;    // [0.0, 1.0]
};

#endif // CSSCOLOR_H
